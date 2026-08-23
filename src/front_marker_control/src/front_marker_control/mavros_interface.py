"""MAVROS raw body-velocity setpoint definitions."""

from mavros_msgs.msg import PositionTarget


def velocity_only_type_mask():
    return (
        PositionTarget.IGNORE_PX
        | PositionTarget.IGNORE_PY
        | PositionTarget.IGNORE_PZ
        | PositionTarget.IGNORE_AFX
        | PositionTarget.IGNORE_AFY
        | PositionTarget.IGNORE_AFZ
        | PositionTarget.IGNORE_YAW
    )


def body_flu_setpoint_components(vx, vy, vz, yaw_rate):
    """Keep ROS-side FLU signs; MAVROS performs the FLU-to-FRD conversion."""
    return float(vx), float(vy), float(vz), float(yaw_rate)
