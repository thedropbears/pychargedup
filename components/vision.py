from components.chassis import Chassis
import wpilib
import wpimath
import math
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
        self.pose_estimator = RobotPoseEstimator(self.FIELD_LAYOUT, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, [(self.camera, Transform3d(Translation3d(-0.35, 0.005, 0.26624), Rotation3d.fromDegrees(0, 0, 180)))])

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
        self.field_pos_obj.setPose(cur_pose)

        std_dev_x = 0.3
        std_dev_y = 0.3
        std_dev_omega = math.inf

        if(self.chassis.chassis_speeds.vx < 0.1):
            std_dev_x = 0.25
        if(self.chassis.chassis_speeds.vy < 0.1):
            std_dev_y = 0.25
        
        if(abs(cur_pose_real.rotation().radians()-rot.radians())< (math.radians(10))):
            std_dev_omega = 0.5

        self.chassis.estimator.addVisionMeasurement(cur_pose, timestamp,(std_dev_x,std_dev_y,std_dev_omega))
        
