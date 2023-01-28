from components.chassis import Chassis
import wpilib
import wpiutil.log
import robotpy_apriltag
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d, Quaternion
from typing import Optional
from magicbot import tunable
import math

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
    X_STD_DEV_CONSTANT = Y_STD_DEV_CONSTANT = 9999.0
    ANGULAR_STD_DEV_CONSTANT = 99999.0

    field: wpilib.Field2d

    data_log: wpiutil.log.DataLog

    enabled = tunable(True)

    def __init__(self) -> None:
        left_rot = Rotation3d(Quaternion(0.2679, -0.1853, 0.0304, -0.9449))
        right_rot = Rotation3d(Quaternion(0.2679, 0.1853, 0.0304, -0.9449))
        # flip to facing forwards
        left_rot.rotateBy(Rotation3d(0, 0, math.pi))
        right_rot.rotateBy(Rotation3d(0, 0, math.pi))
        self.cameras = [  # (Camera object, camera-to-robot transform)
            (
                PhotonCamera(n),
                Transform3d(Translation3d(x, y, z), rot).inverse(),
            )
            for (n, x, y, z, rot) in [
                ("C922_Left", 0.36986, 0.05223, 0.22041, left_rot),
                ("C920_Right", 0.36986, -0.05223, 0.22041, right_rot),
            ]
        ]
        self.last_timestamps = [0] * len(self.cameras)
        self.should_log = False

    def setup(self) -> None:
        self.field_pos_obj = self.field.getObject("vision_pose")
        self.pose_log_entry = wpiutil.log.FloatArrayLogEntry(
            self.data_log, "vision_pose"
        )
        self.pose_error_log_entry = wpiutil.log.FloatArrayLogEntry(
            self.data_log, "vision_error"
        )

    def execute(self) -> None:
        for i, (c, trans) in enumerate(self.cameras):
            # if results didnt see any targets
            if not (results := c.getLatestResult()).hasTargets():
                continue
            # if we have already processed these results
            if (timestamp := results.getTimestamp()) == self.last_timestamps[
                i
            ] and wpilib.RobotBase.isReal():
                continue
            print(timestamp, wpilib.Timer.getFPGATimestamp())
            # if the result is too old
            if abs(wpilib.Timer.getFPGATimestamp() - timestamp) > 0.5:
                continue

            self.last_timestamps[i] = timestamp
            for t in results.getTargets():
                tag_id = t.getFiducialId()
                # tag isnt one thats on the field
                if tag_id < 0 or tag_id > 8:
                    print(f"Invalid ag id {tag_id}")
                    continue
                pose = estimate_pos_from_apriltag(trans, t)
                if pose is None:
                    continue

                if self.enabled:
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
                    cur_pose = self.chassis.get_pose()
                    self.pose_log_entry.append(
                        [pose.X(), pose.Y(), pose.rotation().radians()]
                    )
                    trans_error = cur_pose.translation().distance(pose.translation())
                    rot_error = cur_pose.rotation() - pose.rotation()
                    self.pose_error_log_entry.append([trans_error, rot_error.radians()])
                self.field_pos_obj.setPose(pose)


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
