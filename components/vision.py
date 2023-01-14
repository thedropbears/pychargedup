from components.chassis import Chassis
import wpilib
import wpimath
import robotpy_apriltag
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d

from photonvision import (
    PhotonCamera,
    PhotonUtils,
    LEDMode,
    SimVisionSystem,
    SimVisionTarget,
    RobotPoseEstimator,
    PoseStrategy
)

class Vision:
    chassis: Chassis

    FIELD_LAYOUT = robotpy_apriltag.AprilTagFieldLayout(wpilib.getDeployDirectory() + "/field_layout.json")

    field: wpilib.Field2d

    def __init__(self) -> None:
        self.camera = PhotonCamera("forward_camera")
        self.has_targets = False
        self.last_timestamp = 0
        self.pose_estimator = RobotPoseEstimator(self.FIELD_LAYOUT, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, [(self.camera, Transform3d(Translation3d(-0.35, -0.085, 0.11), Rotation3d.fromDegrees(0, 0, 0)))])

    def setup(self) -> None:
        self.field_pos_obj = self.field.getObject("vision_pose")

    def execute(self) -> None:
        results = self.camera.getLatestResult()
        self.has_targets = self.camera.hasTargets()
        timestamp = results.getTimestamp()

        if not self.has_targets:
            return
        if timestamp == self.last_timestamp and wpilib.RobotBase.isReal():
            return

        self.last_timestamp = timestamp
        self.pose_estimator.update()
        # current pose2d with rotation from vision
        cur_pose_real = self.pose_estimator.getLastPose().toPose2d()
        cur_translation = cur_pose_real.translation()
        # create a new pose that has the translation from the vision and rotation from gyro
        # gets rotation when image was taken
        rot = self.chassis.get_pose_at(timestamp).rotation()
        cur_pose = Pose2d(cur_translation, rot)
        self.field_pos_obj.setPose(cur_pose_real)
        std_dev = 1
        self.chassis.estimator.addVisionMeasurement(cur_pose, timestamp)
        
