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

    FIELD_LAYOUT = robotpy_apriltag.loadAprilTagLayoutField(
        robotpy_apriltag.AprilTagField.k2023ChargedUp
    )
    FORWARD_CAMERA_TRANSFORM = Transform3d(
        Translation3d(-0.35, 0.01, 0.11), Rotation3d.fromDegrees(0, 0, 175)
    )
    STD_DEV_CONSTANT = 5.0
    ANGULAR_STD_DEV_CONSTANT = 3.0
    ZERO_DIVISION_THRESHOLD = 1e-6
    POSE_AMBIGUITY_FACTOR = 5.0
    POSE_AMBIGUITY_THRESHOLD = 0.2

    VELOCITY_SCALING_THRESHOLD = 0.5
    VELOCITY_SCALING_FACTOR = 2

    CONF_EXP_FILTER_ALPHA = 0.8

    field: wpilib.Field2d
    enabled = tunable(True)
    use_resection = tunable(False)

    def __init__(self) -> None:
        self.camera = PhotonCamera("forward_camera")
        self.has_targets = False
        self.targets = [PhotonTrackedTarget]
        self.last_timestamp = 0
        self.confidence_accs = [0.0] * 8

    def setup(self) -> None:
        self.field_pos_obj = self.field.getObject("vision_pose")

    def execute(self) -> None:
        if not self.enabled:
            return
        results = self.camera.getLatestResult()
        self.has_targets = results.hasTargets()
        timestamp = results.getTimestamp()

        if not self.has_targets:
            return
        if timestamp == self.last_timestamp and wpilib.RobotBase.isReal():
            return

        self.last_timestamp = timestamp
        self.targets = results.getTargets()

        estimated_pose = Pose2d()
        std_dev_x = std_dev_y = math.inf
        estimated_poses = [
            p
            for p in (
                estimate_pos_from_apriltag(Vision.FORWARD_CAMERA_TRANSFORM, t)
                for t in self.targets
                if t.getPoseAmbiguity() < Vision.POSE_AMBIGUITY_THRESHOLD
            )
            if p is not None
        ]
        weights = [0.0] * len(self.targets)
        for i, t in enumerate(self.targets):
            decr_tag_id = t.getFiducialId() - 1
            new_confidence = 1.0 - Vision.POSE_AMBIGUITY_FACTOR * t.getPoseAmbiguity()
            weights[i] = self.confidence_accs[decr_tag_id] = (
                Vision.CONF_EXP_FILTER_ALPHA * self.confidence_accs[decr_tag_id]
                + (1 - Vision.CONF_EXP_FILTER_ALPHA) * new_confidence
            )

        if any(w < Vision.ZERO_DIVISION_THRESHOLD for w in weights):
            return
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
            timestamp,
            (f * std_dev_x, f * std_dev_y, f * Vision.ANGULAR_STD_DEV_CONSTANT),
        )


def estimate_pos_from_apriltag(
    cam_trans: Transform3d, target: PhotonTrackedTarget
) -> Optional[Pose2d]:
    tag_id = target.getFiducialId()
    tag_pose = Vision.FIELD_LAYOUT.getTagPose(tag_id)
    if tag_pose is None:
        return None
    return (
        tag_pose.transformBy(target.getBestCameraToTarget().inverse())
        .transformBy(cam_trans.inverse())
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
