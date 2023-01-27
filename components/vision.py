from components.chassis import Chassis
import wpilib
import wpiutil.log
import robotpy_apriltag
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d
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
    X_STD_DEV_CONSTANT = Y_STD_DEV_CONSTANT = 0.4
    ANGULAR_STD_DEV_CONSTANT = 0.5

    field: wpilib.Field2d

    data_log: wpiutil.log.DataLog

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
                ("C922_Left", 0.36986, 0.05223, 0.22041, 0.0, 0, 149.79),
                ("C920_Right", 0.36986, -0.05223, 0.22041, 0.0, 0, -149.79),
            ]
        ]
        self.last_timestamps = [0] * len(self.cameras)

    def setup(self) -> None:
        self.field_pos_obj = self.field.getObject("vision_pose")
        self.pose_log_entry = wpiutil.log.FloatArrayLogEntry(
            self.data_log, "vision_pose"
        )

    def execute(self) -> None:
        if not self.enabled:
            return
        for i, (c, trans) in enumerate(self.cameras):
            # if results didnt see any targets
            if not (results := c.getLatestResult()).hasTargets():
                continue
            # if we have already processed these results
            if (timestamp := results.getTimestamp()) == self.last_timestamps[
                i
            ] and wpilib.RobotBase.isReal():
                continue
            # if the result is too old
            if wpilib.Timer.getFPGATimestamp() - timestamp > 1.0:
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

                self.chassis.estimator.addVisionMeasurement(
                    pose,
                    timestamp,
                    (
                        Vision.X_STD_DEV_CONSTANT,
                        Vision.Y_STD_DEV_CONSTANT,
                        Vision.ANGULAR_STD_DEV_CONSTANT,
                    ),
                )

                self.pose_log_entry.append(
                    [pose.X(), pose.Y(), pose.rotation().radians()], int(timestamp)
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
