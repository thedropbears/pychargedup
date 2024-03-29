import math
import typing
from typing import Optional

import wpilib
import wpiutil.log
from magicbot import tunable
from photonvision import PhotonCamera, PhotonTrackedTarget
from wpimath.geometry import Pose2d, Rotation3d, Transform3d, Translation3d

from components.chassis import Chassis
from utilities.game import apriltag_layout
from utilities.scalers import scale_value
from utilities.functions import clamp


class VisualLocalizer:
    """
    This localizes the robot from AprilTags on the field,
    using information from a single PhotonVision camera.
    """

    add_to_estimator = tunable(False)
    should_log = tunable(False)

    rejected_in_row = tunable(0.0)
    last_pose_z = tunable(0.0, writeDefault=False)

    def __init__(
        self,
        # The name of the camera in PhotonVision.
        name: str,
        # Position of the camera relative to the center of the robot
        pos: Translation3d,
        # The camera rotation.
        rot: Rotation3d,
        field: wpilib.Field2d,
        data_log: wpiutil.log.DataLog,
        chassis: Chassis,
    ) -> None:
        self.camera = PhotonCamera(name)
        self.camera_to_robot = Transform3d(pos, rot).inverse()
        self.last_timestamp = -1

        self.field_pos_obj = field.getObject("vision_pose_" + name)
        self.pose_log_entry = wpiutil.log.DoubleArrayLogEntry(
            data_log, "vision_pose_" + name
        )

        self.chassis = chassis

    def on_disable(self) -> None:
        self.add_to_estimator = False

    def on_enable(self) -> None:
        self.add_to_estimator = True

    def execute(self) -> None:
        # stop warnings in simulation
        if wpilib.RobotBase.isSimulation():
            return
        # if results didn't see any targets
        if not (results := self.camera.getLatestResult()).hasTargets():
            return

        # if we have already processed these results
        timestamp = results.getTimestamp()
        if timestamp == self.last_timestamp and wpilib.RobotBase.isReal():
            return
        self.last_timestamp = timestamp

        # old results cause pose estimator to crash and aren't very useful anyway
        if abs(wpilib.Timer.getFPGATimestamp() - timestamp) > 0.5:
            return

        for target in results.getTargets():
            poses = estimate_poses_from_apriltag(self.camera_to_robot, target)
            distance_to_tag = target.getBestCameraToTarget().translation().norm()
            if poses is None:
                # tag doesn't exist
                continue
            best_pose, alt_pose, self.last_pose_z = poses
            pose = choose_pose(
                best_pose,
                alt_pose,
                self.chassis.get_pose(),
                target.getPoseAmbiguity(),
            )

            # filter out likely bad targets
            if target.getPoseAmbiguity() > 0.25:
                continue

            self.field_pos_obj.setPose(pose)
            change = self.chassis.get_pose().translation().distance(pose.translation())
            if change > 1.0:
                self.rejected_in_row += 1
                if self.rejected_in_row < 10:
                    continue
            else:
                self.rejected_in_row //= 2

            if self.add_to_estimator:
                std_apriltag = clamp(
                    scale_value(distance_to_tag, 3.0, 6.0, 0.5, 3.0), 0.5, 3
                )
                self.chassis.estimator.addVisionMeasurement(
                    pose,
                    timestamp,
                    (std_apriltag, std_apriltag, std_apriltag / 3),
                )

            if self.should_log:
                ground_truth_pose = self.chassis.get_pose()
                trans_error1: float = ground_truth_pose.translation().distance(
                    best_pose.translation()
                )
                trans_error2: float = ground_truth_pose.translation().distance(
                    alt_pose.translation()
                )
                rot_error1: float = (  # type: ignore
                    ground_truth_pose.rotation() - best_pose.rotation()
                ).radians()
                rot_error2: float = (  # type: ignore
                    ground_truth_pose.rotation() - alt_pose.rotation()
                ).radians()
                skew = get_target_skew(target)

                self.pose_log_entry.append(
                    [
                        best_pose.x,
                        best_pose.y,
                        typing.cast(float, best_pose.rotation().radians()),
                        trans_error1,  # error of main pose
                        rot_error1,
                        alt_pose.x,
                        alt_pose.y,
                        typing.cast(float, alt_pose.rotation().radians()),
                        trans_error2,
                        rot_error2,
                        ground_truth_pose.x,
                        ground_truth_pose.y,
                        target.getYaw(),
                        skew,
                        target.getPoseAmbiguity(),
                        target.getArea(),
                        target.getFiducialId(),
                    ]
                )


def estimate_poses_from_apriltag(
    cam_to_robot: Transform3d, target: PhotonTrackedTarget
) -> Optional[tuple[Pose2d, Pose2d, float]]:
    tag_id = target.getFiducialId()
    tag_pose = apriltag_layout.getTagPose(tag_id)
    if tag_pose is None:
        return None

    best_pose = tag_pose.transformBy(
        target.getBestCameraToTarget().inverse()
    ).transformBy(cam_to_robot)
    alternate_pose = (
        tag_pose.transformBy(target.getAlternateCameraToTarget().inverse())
        .transformBy(cam_to_robot)
        .toPose2d()
    )
    return best_pose.toPose2d(), alternate_pose, best_pose.z


def get_target_skew(target: PhotonTrackedTarget):
    tag_to_cam = target.getBestCameraToTarget().inverse()
    return math.atan2(tag_to_cam.y, tag_to_cam.x)


def choose_pose(
    best_pose: Pose2d, alternate_pose: Pose2d, cur_robot: Pose2d, ambiguity: float
):
    """Picks either the best or alternate pose estimate"""
    best_dist = best_pose.translation().distance(cur_robot.translation())
    best_preferance = 1.2
    alternate_dist = (
        alternate_pose.translation().distance(cur_robot.translation()) * best_preferance
    )

    if best_dist < alternate_dist:
        return best_pose
    else:
        return alternate_pose
