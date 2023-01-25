from components.chassis import Chassis
import wpilib
import robotpy_apriltag
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d, Rotation2d
from typing import Optional
import math
from magicbot import tunable

from photonvision import (
    PhotonCamera,
    PhotonTrackedTarget,
)


class Vision:
    chassis: Chassis

    FIELD_LAYOUT = (
        robotpy_apriltag.loadAprilTagLayoutField(
            robotpy_apriltag.AprilTagField.k2023ChargedUp
        )
        if False
        else robotpy_apriltag.AprilTagFieldLayout(
            wpilib.getDeployDirectory() + "/test_field_layout.json"
        )
    )
    STD_DEV_CONSTANT = 5.0
    ANGULAR_STD_DEV_CONSTANT = 3.0
    ZERO_DIVISION_THRESHOLD = 1e-6
    POSE_AMBIGUITY_FACTOR = 2.5
    POSE_AMBIGUITY_THRESHOLD = 0.4

    VELOCITY_SCALING_FACTOR = 2

    CONF_EXP_FILTER_ALPHA = 0.8

    field: wpilib.Field2d
    enabled = tunable(True)

    def __init__(self) -> None:
        self.cameras = [  # (Camera object, camera-to-robot transform)
            (
                PhotonCamera(n),
                Transform3d(
                    Translation3d(x, y, z), Rotation3d.fromDegrees(roll, pitch, yaw)
                ).inverse(),
            )
            for (n, x, y, z, roll, pitch, yaw) in [
                ("camera_1", -0.35, 0.01, 0.11, 0.0, 0.0, 175),
                ("camera_2", -0.35, 0.01, 0.11, 0.0, 0.0, 175),
            ]
        ]
        self.timestamps = [0] * len(self.cameras)
        self.confidence_accs = [0.0] * 8

    def setup(self) -> None:
        self.field_pos_obj = self.field.getObject("vision_pose")

    def execute(self) -> None:
        if not self.enabled:
            return
        estimated_poses = []
        weights = []
        for i, (c, trans) in enumerate(self.cameras):
            if not (results := c.getLatestResult()).hasTargets():
                continue
            if (timestamp := results.getTimestamp()) == self.timestamps[
                i
            ] and wpilib.RobotBase.isReal():
                continue
            self.timestamps[i] = timestamp
            for t in results.getTargets():
                tag_id = t.getFiducialId()
                if tag_id < 0 or tag_id > 8:
                    print(f"Invalid ag id {tag_id}")
                    continue
                new_confidence = max(
                    1.0 - Vision.POSE_AMBIGUITY_FACTOR * t.getPoseAmbiguity(), 0.0
                )
                weight = self.confidence_accs[tag_id - 1] = (
                    Vision.CONF_EXP_FILTER_ALPHA * self.confidence_accs[tag_id - 1]
                    + (1 - Vision.CONF_EXP_FILTER_ALPHA) * new_confidence
                )
                if weight > Vision.ZERO_DIVISION_THRESHOLD and not (
                    (pos := estimate_pos_from_apriltag(trans, t)) is None
                ):
                    estimated_poses.append(pos)
                    weights.append(weight)

        if len(estimated_poses) == 0:
            return
        estimated_pose = Pose2d()
        std_dev_x = std_dev_y = math.inf

        points = [(p.x, p.y, w) for (p, w) in zip(estimated_poses, weights)]
        mx, my, std_dev_x, std_dev_y = weighted_point_cloud_centroid(points)
        rotation_unit_vectors = [
            (p.rotation().cos(), p.rotation().sin()) for p in estimated_poses
        ]
        accx = accy = 0.0
        for (x, y), w in zip(rotation_unit_vectors, weights):
            accx += x * w
            accy += y * w
        f = 1 / math.hypot(accx, accy)
        estimated_pose = Pose2d(mx, my, Rotation2d(accx * f, accy * f))

        self.field_pos_obj.setPose(estimated_pose)

        v = math.hypot(self.chassis.imu.getVelocityX(), self.chassis.imu.getVelocityY())
        f = (
            1.0
            + max(v - Vision.VELOCITY_SCALING_THRESHOLD, 0.0)
            * Vision.VELOCITY_SCALING_FACTOR
        )  # Start trusting vision less if the robot moves fast

        self.chassis.estimator.addVisionMeasurement(
            estimated_pose,
            min(self.timestamps),
            (f * std_dev_x, f * std_dev_y, f * Vision.ANGULAR_STD_DEV_CONSTANT),
        )


def estimate_pos_from_apriltag(
    cam_to_robot: Transform3d, target: PhotonTrackedTarget
) -> Optional[Pose2d]:
    tag_id = target.getFiducialId()
    tag_pose = Vision.FIELD_LAYOUT.getTagPose(tag_id)
    if tag_pose is None:
        return None
    return (
        tag_pose.transformBy(target.getBestCameraToTarget().inverse())
        .transformBy(cam_to_robot)
        .toPose2d()
    )


def point_cloud_centroid(
    points: list[tuple[float, float]]
) -> tuple[float, float, float, float]:
    f = 1.0 / len(points)
    accx = accy = 0.0
    for (x, y) in points:
        accx += x
        accy += y
    mx = accx * f
    my = accy * f
    accx = accy = 0.0
    for (x, y) in points:
        dx = x - mx
        dy = y - my
        accx += dx * dx
        accy += dy * dy
    return (
        mx,
        my,
        Vision.STD_DEV_CONSTANT * f + math.sqrt(accx * f),
        Vision.STD_DEV_CONSTANT * f + math.sqrt(accy * f),
    )


def weighted_point_cloud_centroid(
    points: list[tuple[float, float, float]]
) -> tuple[float, float, float, float]:
    accx = accy = accw = 0.0
    for (x, y, w) in points:
        accx += x * w
        accy += y * w
        accw += w
    f = 1.0 / accw
    mx = accx * f
    my = accy * f
    accx = accy = accw = 0.0
    for (x, y, w) in points:
        dx = x - mx
        dy = y - my
        accx += (dx * dx) * w
        accy += (dy * dy) * w
    return (
        mx,
        my,
        Vision.STD_DEV_CONSTANT * f + math.sqrt(accx * f),
        Vision.STD_DEV_CONSTANT * f + math.sqrt(accy * f),
    )
