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
        self.timestamp = 0
        self.last_latency = 0
        self.pose_estimator = RobotPoseEstimator(self.FIELD_LAYOUT, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, [(self.camera, Transform3d(Translation3d(0.5, 0, 0.5), Rotation3d.fromDegrees(0, 0, 0)))])
        self.last_pose = Pose2d()

    def setup(self) -> None:
        self.field_pos_obj = self.field.getObject("vision_pose")

    def execute(self) -> None:
        results = self.camera.getLatestResult()
        self.has_targets = self.camera.hasTargets()
        self.timestamp = wpilib.Timer.getFPGATimestamp() - results.getLatency()
        if not self.has_targets:
            return
        if results.getLatency() == self.last_latency and wpilib.RobotBase.isReal():
            return
        self.pose_estimator.update()
        self.last_pose = self.pose_estimator.getLastPose().toPose2d()
        self.field_pos_obj.setPose(self.last_pose)
        std_dev = 1
        self.chassis.estimator.addVisionMeasurement(self.last_pose, self.timestamp, [std_dev, std_dev, 1])
        
