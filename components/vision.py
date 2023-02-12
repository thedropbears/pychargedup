import math
from components.chassis import Chassis
import wpilib
import wpiutil.log
import robotpy_apriltag
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d, Quaternion
from typing import Optional
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

    # TBD
    X_STD_DEV_CONSTANT = Y_STD_DEV_CONSTANT = 2.5
    ANGULAR_STD_DEV_CONSTANT = 1.0

    field: wpilib.Field2d

    data_log: wpiutil.log.DataLog

    add_to_estimator = tunable(True)

    def __init__(self) -> None:
        left_rot = Rotation3d(
            Quaternion(
                -0.2156163454055786,
                0.0850898027420044,
                0.018863987177610397,
                0.9725808501243591,
            )
        )
        right_rot = Rotation3d(
            Quaternion(
                0.21561579406261444,
                0.08508981764316559,
                -0.018863940611481667,
                0.9725809693336487,
            )
        )
        self.cameras = [  # (Camera object, camera-to-robot transform)
            (
                PhotonCamera(n),
                Transform3d(Translation3d(x, y, z), rot).inverse(),
            )
            for (n, x, y, z, rot) in [
                ("cam_port", -0.35001, 0.06583, 0.44971, left_rot),
                ("cam_starboard", -0.35001, -0.06583, 0.44971, right_rot),
            ]
        ]
        self.last_timestamps = [0] * len(self.cameras)
        self.should_log = False
        # amount of tags that have produced unreasonable estimates in a row
        self.limited_in_row = 0

    def setup(self) -> None:
        self.field_pos_obj_left = self.field.getObject("vision_pose_left_cam")
        self.field_pos_obj_right = self.field.getObject("vision_pose_right_cam")
        self.pose_log_entry = wpiutil.log.FloatArrayLogEntry(
            self.data_log, "vision_pose"
        )

    def execute(self) -> None:
        for cam_idx, (camera, trans) in enumerate(self.cameras):
            # if results didnt see any targets
            if not (results := camera.getLatestResult()).hasTargets():
                continue

            # if we have already processed these results
            timestamp = results.getTimestamp()
            if timestamp == self.last_timestamps[cam_idx] and wpilib.RobotBase.isReal():
                continue
            self.last_timestamps[cam_idx] = timestamp

            # old results cause pose estimator to crash and arent very useful anyway
            if abs(wpilib.Timer.getFPGATimestamp() - timestamp) > 0.5:
                continue

            for target in results.getTargets():
                poses = estimate_poses_from_apriltag(trans, target)
                # tag dosent exist
                if poses is None:
                    continue
                pose = choose_pose(
                    poses[0],
                    poses[1],
                    self.chassis.get_pose(),
                    target.getPoseAmbiguity(),
                )

                # filter out likely bad targets
                if target.getPoseAmbiguity() > 0.25 or target.getYaw() > 20:
                    continue

                change = (
                    self.chassis.get_pose().translation().distance(pose.translation())
                )
                if change > 2.0:
                    if self.limited_in_row < 50:
                        continue
                    self.limited_in_row += 1
                else:
                    self.limited_in_row /= 2

                if self.add_to_estimator:
                    self.chassis.estimator.addVisionMeasurement(
                        pose,
                        timestamp,
                        (
                            Vision.X_STD_DEV_CONSTANT,
                            Vision.Y_STD_DEV_CONSTANT,
                            Vision.ANGULAR_STD_DEV_CONSTANT,
                        ),
                    )
                if self.should_log:
                    ground_truth_pose = self.chassis.get_pose()
                    trans_error1 = ground_truth_pose.translation().distance(
                        poses[0].translation()
                    )
                    trans_error2 = ground_truth_pose.translation().distance(
                        poses[1].translation()
                    )
                    rot_error1 = (
                        ground_truth_pose.rotation() - poses[0].rotation()
                    ).radians()
                    rot_error2 = (
                        ground_truth_pose.rotation() - poses[1].rotation()
                    ).radians()
                    skew = get_target_skew(target)

                    self.pose_log_entry.append(
                        [
                            poses[0].X(),
                            poses[0].Y(),
                            poses[0].rotation().radians(),
                            trans_error1,  # error of main pose
                            rot_error1,
                            poses[1].X(),
                            poses[1].Y(),
                            poses[1].rotation().radians(),
                            trans_error2,
                            rot_error2,
                            ground_truth_pose.x,
                            ground_truth_pose.y,
                            target.getYaw(),
                            skew,
                            target.getPoseAmbiguity(),
                            target.getArea(),
                            target.getFiducialId(),
                            cam_idx,  # camera num
                        ]
                    )
                if cam_idx == 0:
                    self.field_pos_obj_left.setPose(pose)
                else:
                    self.field_pos_obj_right.setPose(pose)


def estimate_poses_from_apriltag(
    cam_to_robot: Transform3d, target: PhotonTrackedTarget
) -> Optional[tuple[Pose2d, Pose2d]]:
    tag_id = target.getFiducialId()
    tag_pose = Vision.FIELD_LAYOUT.getTagPose(tag_id)
    if tag_pose is None:
        return None

    best_pose = (
        tag_pose.transformBy(target.getBestCameraToTarget().inverse())
        .transformBy(cam_to_robot)
        .toPose2d()
    )
    alternate_pose = (
        tag_pose.transformBy(target.getAlternateCameraToTarget().inverse())
        .transformBy(cam_to_robot)
        .toPose2d()
    )
    return best_pose, alternate_pose


def get_target_skew(target: PhotonTrackedTarget):
    tag_to_cam = target.getBestCameraToTarget().inverse()
    return math.atan2(tag_to_cam.y, tag_to_cam.x)


def choose_pose(
    best_pose: Pose2d, alternate_pose: Pose2d, cur_robot: Pose2d, ambiguity: float
):
    """Picks either the best or alternate pose estimate"""
    return best_pose
