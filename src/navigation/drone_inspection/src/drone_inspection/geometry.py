"""Fixed upright shelf geometry and shortest in-batch visit order."""
import cv2
import numpy as np
from drone_navigation.core import heading, rotation, rz

STANDOFF = .8


def scan_target(tag_position, tag_rotation, label, reference, camera_lever, yaw_map):
    """Place the optical center in front of the paper with the route heading in drone_map."""
    paper = tag_position - tag_rotation @ np.array([label['tag_offset_x_meters'],
                                                    label['tag_offset_y_meters'], 0.])
    normal = tag_rotation[:, 2]
    camera = paper + STANDOFF * normal
    r, t = reference
    yaw_odom = heading(r.T, yaw_map)
    point_odom = r.T @ (camera - t) - rz(yaw_odom) @ camera_lever
    return r @ point_odom + t, point_odom, yaw_map, camera, paper


def shortest_order(start, points, finish):
    """Return the ID order and total 3D Euclidean distance, including the fixed finish."""
    keys = sorted(points)
    n = len(keys)
    if n > 10:
        raise ValueError('more than 10 distinct tags at one observation point')
    if not n:
        return (), float(np.linalg.norm(start - finish))
    p = np.asarray([points[key] for key in keys])
    distances = np.linalg.norm(p[:, None] - p[None, :], axis=2)
    initial = np.linalg.norm(p - start, axis=1)
    final = np.linalg.norm(p - finish, axis=1)
    dp = {(1 << i, i): (initial[i], (keys[i],)) for i in range(n)}
    for mask in range(1, 1 << n):
        for i in range(n):
            if not mask & (1 << i) or mask == 1 << i:
                continue
            previous = mask ^ (1 << i)
            dp[mask, i] = min((dp[previous, j][0] + distances[j, i],
                               dp[previous, j][1] + (keys[i],))
                              for j in range(n) if previous & (1 << j))
    distance, order = min((dp[(1 << n) - 1, i][0] + final[i], dp[(1 << n) - 1, i][1])
                          for i in range(n))
    return order, float(distance)

def paper_axes(yaw):
    normal = np.array([-np.cos(yaw), -np.sin(yaw), 0.])
    up = np.array([0., 0., 1.])
    return np.column_stack((np.cross(up, normal), up, normal))


def fit_center(pixels, size, camera_axes, intrinsics, distortion):
    pixels = np.asarray(pixels, dtype=np.float64)
    if pixels.shape != (4, 2) or not np.isfinite(pixels).all() or size <= 0:
        raise ValueError('invalid tag corners or size')
    points = .5 * size * np.array([[-1.,-1.,0.], [1.,-1.,0.], [1.,1.,0.], [-1.,1.,0.]])
    offsets = points @ camera_axes.T
    rays = cv2.undistortPoints(pixels[:, None], intrinsics, distortion).reshape(4, 2)
    design = np.tile([[1.,0.,0.], [0.,1.,0.]], (4,1))
    design[:,2] = -rays.ravel()
    rhs = (rays * offsets[:,2,None] - offsets[:,:2]).ravel()
    center, _, rank, _ = np.linalg.lstsq(design, rhs, rcond=None)
    if rank != 3:
        raise ValueError('degenerate corner rays')
    rvec = cv2.Rodrigues(camera_axes)[0]
    # The rotation is constant. Only columns 3:6 of OpenCV's Jacobian are translation.
    for iteration in range(7):
        if not np.isfinite(center).all() or np.min(center[2] + offsets[:,2]) <= 0:
            raise ValueError('invalid or nonpositive tag depth')
        projected, jacobian = cv2.projectPoints(points, rvec, center, intrinsics, distortion)
        residual = (pixels - projected.reshape(4,2)).ravel()
        rms = float(np.linalg.norm(residual) / 2.)
        if not np.isfinite(rms):
            raise ValueError('nonfinite reprojection')
        if iteration == 6:
            break
        step, _, rank, _ = np.linalg.lstsq(jacobian[:,3:6], residual, rcond=None)
        if rank != 3:
            raise ValueError('degenerate translation fit')
        if np.linalg.norm(step) < 1e-8:
            break
        center += step
    return center, rms


def fit_observation(pixels, stamp_pose, reference, cam_rot, lever, axes, size, k, d):
    r, t = reference
    body_map = r @ rotation(stamp_pose.quaternion)
    camera_map = r @ stamp_pose.position + t + body_map @ lever
    view = body_map @ cam_rot
    relative_axes = view.T @ axes
    center, rms = fit_center(pixels, size, relative_axes, k, d)
    return center, relative_axes, camera_map + view @ center, camera_map, rms
