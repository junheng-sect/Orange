#!/usr/bin/env python3
"""Collect an exact-source-timestamp right-camera baseline/improved A/B report."""

from __future__ import print_function

import json
import math
import threading

from front_marker_tracking.msg import HomogeneousTransform
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String


class AbReport(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.duration = float(rospy.get_param('~duration_sec', 60.0))
        self.output_path = str(
            rospy.get_param(
                '~output_path', '/tmp/front_marker_right_ab_report.json'
            )
        )
        self.image_stamps = []
        self.transforms = {'baseline': [], 'improved': []}
        self.processing = {'baseline': [], 'improved': []}
        self.diagnostics = {'baseline': [], 'improved': []}

        rospy.Subscriber(
            rospy.get_param('~image_topic'),
            Image,
            self.image_callback,
            queue_size=200,
        )
        for name in ('baseline', 'improved'):
            rospy.Subscriber(
                rospy.get_param('~%s_transform_topic' % name),
                HomogeneousTransform,
                self.transform_callback,
                callback_args=name,
                queue_size=200,
            )
            rospy.Subscriber(
                rospy.get_param('~%s_processing_topic' % name),
                Float64,
                self.processing_callback,
                callback_args=name,
                queue_size=200,
            )
            rospy.Subscriber(
                rospy.get_param('~%s_diagnostic_topic' % name),
                String,
                self.diagnostic_callback,
                callback_args=name,
                queue_size=200,
            )

    def image_callback(self, message):
        with self.lock:
            self.image_stamps.append(message.header.stamp.to_nsec())

    def transform_callback(self, message, name):
        matrix = np.asarray(message.matrix, dtype=np.float64).reshape(4, 4)
        with self.lock:
            self.transforms[name].append(
                (message.header.stamp.to_nsec(), matrix)
            )

    def processing_callback(self, message, name):
        with self.lock:
            self.processing[name].append(float(message.data))

    def diagnostic_callback(self, message, name):
        with self.lock:
            self.diagnostics[name].append(str(message.data))

    @staticmethod
    def yaw_from_matrix(matrix):
        marker_facing = -matrix[:3, 2]
        return math.atan2(float(marker_facing[0]), float(marker_facing[2]))

    @staticmethod
    def prefix_counts(messages):
        prefixes = (
            'DETECTED',
            'WAIT_MARKER',
            'REJECTED_CONTOUR_RAW',
            'REJECTED_CONTOUR_REFINED',
            'REJECTED_SUBPIX',
            'REJECTED_IPPE',
            'REJECTED_REPROJECTION',
            'REJECTED_POSE',
        )
        counts = dict((prefix, 0) for prefix in prefixes)
        counts['OTHER'] = 0
        for text in messages:
            matched = False
            for prefix in prefixes:
                if text.startswith(prefix):
                    counts[prefix] += 1
                    matched = True
                    break
            if not matched:
                counts['OTHER'] += 1
        return counts

    def summarize(self, name, image_count):
        records = self.transforms[name]
        result = {
            'image_count': int(image_count),
            'pose_count': int(len(records)),
            'coverage_percent': 100.0 * len(records) / max(1, image_count),
            'diagnostic_counts': self.prefix_counts(self.diagnostics[name]),
        }
        processing = np.asarray(self.processing[name], dtype=np.float64)
        if processing.size:
            result['processing_ms'] = {
                'mean': float(np.mean(processing)),
                'p95': float(np.percentile(processing, 95)),
                'max': float(np.max(processing)),
            }
        if len(records) < 2:
            return result

        stamps = np.asarray([item[0] for item in records], dtype=np.float64) * 1e-9
        matrices = np.stack([item[1] for item in records])
        z = matrices[:, 2, 3]
        yaw = np.unwrap(
            np.asarray([self.yaw_from_matrix(matrix) for matrix in matrices])
        )
        yaw_deg = np.degrees(yaw)
        dz_mm = np.abs(np.diff(z)) * 1000.0
        dyaw_deg = np.abs(np.diff(yaw_deg))
        gaps = np.diff(stamps)
        result.update(
            {
                'pose_hz': float((len(records) - 1) / (stamps[-1] - stamps[0])),
                'max_pose_gap_sec': float(np.max(gaps)),
                'unique_xyz_1e12': int(
                    len(
                        {
                            tuple(np.round(matrix[:3, 3], 12))
                            for matrix in matrices
                        }
                    )
                ),
                'z_m': {
                    'mean': float(np.mean(z)),
                    'std_mm': float(np.std(z) * 1000.0),
                    'ptp_mm': float(np.ptp(z) * 1000.0),
                    'max_step_mm': float(np.max(dz_mm)),
                    'step_gt_2mm': int(np.count_nonzero(dz_mm > 2.0)),
                    'step_gt_5mm': int(np.count_nonzero(dz_mm > 5.0)),
                },
                'yaw_deg': {
                    'mean': float(np.mean(yaw_deg)),
                    'std': float(np.std(yaw_deg)),
                    'ptp': float(np.ptp(yaw_deg)),
                    'max_step': float(np.max(dyaw_deg)),
                    'step_gt_0_2deg': int(np.count_nonzero(dyaw_deg > 0.2)),
                    'step_gt_0_5deg': int(np.count_nonzero(dyaw_deg > 0.5)),
                    'step_gt_1deg': int(np.count_nonzero(dyaw_deg > 1.0)),
                },
            }
        )
        return result

    def build_report(self):
        with self.lock:
            image_count = len(self.image_stamps)
            baseline = list(self.transforms['baseline'])
            improved = list(self.transforms['improved'])
            report = {
                'duration_sec': self.duration,
                'baseline': self.summarize('baseline', image_count),
                'improved': self.summarize('improved', image_count),
            }

        baseline_by_stamp = dict(baseline)
        improved_by_stamp = dict(improved)
        paired_stamps = sorted(set(baseline_by_stamp) & set(improved_by_stamp))
        report['paired_count'] = len(paired_stamps)
        if paired_stamps:
            z_delta = []
            yaw_delta = []
            for stamp in paired_stamps:
                first = baseline_by_stamp[stamp]
                second = improved_by_stamp[stamp]
                z_delta.append((second[2, 3] - first[2, 3]) * 1000.0)
                delta = self.yaw_from_matrix(second) - self.yaw_from_matrix(first)
                yaw_delta.append(
                    math.degrees(math.atan2(math.sin(delta), math.cos(delta)))
                )
            report['improved_minus_baseline'] = {
                'z_mm_mean': float(np.mean(z_delta)),
                'z_mm_std': float(np.std(z_delta)),
                'yaw_deg_mean': float(np.mean(yaw_delta)),
                'yaw_deg_std': float(np.std(yaw_delta)),
            }
        return report

    def run(self):
        rospy.loginfo('Collecting front/right A/B data for %.1f seconds', self.duration)
        rospy.sleep(self.duration)
        report = self.build_report()
        rendered = json.dumps(report, indent=2, sort_keys=True)
        print(rendered)
        if self.output_path:
            with open(self.output_path, 'w') as output:
                output.write(rendered + '\n')
            rospy.loginfo('A/B report written to %s', self.output_path)


def main():
    rospy.init_node('front_marker_right_ab_report')
    reporter = AbReport()
    reporter.run()


if __name__ == '__main__':
    main()
