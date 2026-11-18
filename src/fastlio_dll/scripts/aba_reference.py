"""Register stationary raw scans to the tracking map at the LiDAR IMU origin."""
import itertools
import json
import math
import struct

import cv2
import numpy as np
import yaml
from drone_mapping import assemble, mapio
from drone_relocalization.geometry import angle, quat_R, rotz, transform, yaw
from drone_relocalization.matching import MapModel
from drone_relocalization.settings import load


def voxel(points, size):
    return mapio.voxel_down(points, np.zeros(len(points), np.float32), size)[0]


def static_cloud(bag, record_bounds, start, stop, calibration):
    import rospy
    with open(calibration) as stream:
        params = yaml.safe_load(stream)
    extrinsic = params['mapping']
    rotation = np.array(extrinsic['extrinsic_R']).reshape(3, 3)
    translation = np.array(extrinsic['extrinsic_T'])
    chunks = []
    for _, raw, _ in bag.read_messages(topics=['/livox/lidar'], raw=True,
            start_time=rospy.Time.from_sec(record_bounds[0] - .2),
            end_time=rospy.Time.from_sec(record_bounds[1] + .2)):
        data = raw[1]
        sec, nsec = struct.unpack_from('<II', data, 4)
        stamp = sec + nsec * 1e-9
        offset = 32 + struct.unpack_from('<I', data, 12)[0]
        count = struct.unpack_from('<I', data, offset)[0]
        points = np.frombuffer(data, assemble.PT, count=count, offset=offset+4)
        when = stamp + points['t'] * 1e-9
        tag = points['tag'] & 0x30
        points = points[(when >= start) & (when < stop) & (points['line'] < params['preprocess']['scan_line'])
                        & ((tag == 0) | (tag == 0x10))]
        xyz = np.column_stack([points[k] for k in 'xyz'])
        distance = np.linalg.norm(xyz, axis=1)
        xyz = xyz[np.isfinite(xyz).all(axis=1) & (distance > params['preprocess']['blind']) & (distance < 22)]
        if len(xyz):
            chunks.append(xyz)
    if not chunks:
        raise ValueError('B window has no usable raw points')
    points = voxel(np.concatenate(chunks), .16)
    if len(points) < 250:
        raise ValueError('B cloud has fewer than 250 independent voxels')
    return points @ rotation.T + translation


class LocalPlanes:
    """Bounded FLANN index around B; do not apply the startup map's crop around A."""
    def __init__(self, points):
        self.points = np.ascontiguousarray(points, dtype=np.float32)
        self.index = cv2.flann_Index(self.points, dict(algorithm=1, trees=4))
        neighbors, _ = self.index.knnSearch(self.points, 10, params=dict(checks=48))
        patches = self.points[neighbors]
        patches -= patches.mean(axis=1, keepdims=True)
        eigenvalues, axes = np.linalg.eigh(patches.transpose(0, 2, 1) @ patches)
        self.normal = axes[:, :, 0]
        self.planar = eigenvalues[:, 0] < .1 * eigenvalues.sum(axis=1)

    def residual(self, points, max_distance):
        index, squared = self.index.knnSearch(np.ascontiguousarray(points, dtype=np.float32),
                                               1, params=dict(checks=32))
        index = index[:, 0]
        normals = self.normal[index]
        residual = np.einsum('ij,ij->i', points - self.points[index], normals)
        residual[(squared[:, 0] >= max_distance**2) | ~self.planar[index]] = np.inf
        return residual, normals


class CheckpointICP(MapModel):
    def __init__(self, map_dir, center):
        points, _ = mapio.read_pcd(map_dir / 'map_world.pcd')
        nearby = points[np.linalg.norm(points - center, axis=1) < 25]
        if len(nearby) < 250:
            raise ValueError('Too few map points around B')
        cv2.setRNGSeed(0)
        self.nn = LocalPlanes(voxel(nearby, .12))
        self.cfg = load(str(map_dir / 'outdoor_reloc.yaml'))
        # Allow coarse starting offsets, then use the same fine point-to-plane solver.
        self.fine_reach = self.cfg['nn_reach']


def in_survey(points, crop):
    mask = np.ones(len(points), bool)
    for axis, (lo, hi) in enumerate(zip(crop['requested_min'], crop['requested_max'])):
        if lo is not None:
            mask &= points[:, axis] >= lo
        if hi is not None:
            mask &= points[:, axis] <= hi
    if crop.get('xy_polygon_m'):
        vertices = np.asarray(crop['xy_polygon_m'])
        edges = np.roll(vertices, -1, axis=0) - vertices
        sign = np.sign(np.cross(edges[0], edges[1]))
        for vertex, edge in zip(vertices, edges):
            mask &= sign * np.cross(edge, points[:, :2] - vertex) >= 0
    return mask


def register(cloud, map_dir, translation, rotation):
    model = CheckpointICP(map_dir, translation)
    raw_count = len(cloud)
    meta = json.loads((map_dir / 'meta.json').read_text())
    crop = meta.get('shared_outdoor_crop')
    if crop:
        if not in_survey(translation[None, :], crop)[0]:
            raise ValueError('B hint lies outside the tracking map crop')
        cloud = cloud[in_survey(cloud @ rotation.T + translation, crop)]
    radius = model.cfg['max_range']
    if radius:
        cloud = cloud[np.linalg.norm(cloud, axis=1) <= radius]
    if len(cloud) < 250:
        raise ValueError('Too few raw scan voxels within the tracking map survey volume')
    trials = []
    # Opposing position and heading perturbations test the local convergence basin.
    offsets = [(0, 0, 0, 0), (.35, 0, .1, 3), (-.35, 0, -.1, -3),
               (0, .35, -.1, 3), (0, -.35, .1, -3)]
    for dx, dy, dz, degrees in offsets:
        start = translation + [dx, dy, dz]
        orient = rotz(math.radians(degrees)) @ rotation
        model.cfg['nn_reach'] = .65
        coarse = model.icp(cloud, [*start, yaw(orient)], R0=orient, iters=22)
        model.cfg['nn_reach'] = model.fine_reach
        result = model.icp(cloud, [*coarse['t'], yaw(coarse['R'])], R0=coarse['R'])
        trials.append(dict(start=transform(orient, start), pose=transform(result['R'], result['t']),
                           **{k: v for k, v in result.items() if k not in ('R', 't')}))
    pairs = list(itertools.combinations(trials, 2))
    xyz = [np.array(row['pose']['translation']) for row in trials]
    spread = max(np.linalg.norm(a-b) for a, b in itertools.combinations(xyz, 2))
    angular = max(math.degrees(angle(quat_R(a['pose']['q_xyzw']) @ quat_R(b['pose']['q_xyzw']).T))
                  for a,b in pairs)
    cfg = model.cfg
    failures = []
    if spread > cfg['consensus_m'] or angular > cfg['consensus_deg']:
        failures.append('multistart_disagreement')
    for key, threshold in [('inlier', cfg['inlier_ok']), ('fitness_v', cfg['fit_v_ok']),
                           ('degen_xy', cfg['degen_ok']), ('rank_ratio', 1e-6)]:
        if min(row[key] for row in trials) < threshold:
            failures.append(key)
    if max(row['rmse'] for row in trials) > cfg['rmse_ok']:
        failures.append('rmse')
    if crop and not all(in_survey(np.array(x['pose']['translation'])[None, :], crop)[0] for x in trials):
        failures.append('solution_outside_map_crop')
    best = max(trials, key=lambda row: (row['support_v'], -row['rmse']))
    return dict(valid=not failures, reasons=failures, pose=best['pose'] if not failures else None,
                raw_voxels=raw_count, cloud_points=len(cloud), spread_m=spread, spread_deg=angular, trials=trials)
