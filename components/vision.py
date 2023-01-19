from components.chassis import Chassis
import wpilib
import robotpy_apriltag
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d
from typing import Optional
from math import sqrt

from photonvision import (
    PhotonCamera,
    PhotonTrackedTarget,
)


class Vision:
    chassis: Chassis

    FIELD_LAYOUT = robotpy_apriltag.AprilTagFieldLayout(
        wpilib.getDeployDirectory() + "/test_field_layout.json"
    )
    FORWARD_CAMERA_TRANSFORM = Transform3d(
        Translation3d(-0.35, 0.01, 0.11), Rotation3d.fromDegrees(0, 0, 180)
    )
    STD_DEV_CONSTANT = 1.0
    ZERO_DIVISION_THRESHOLD = 0.0001

    field: wpilib.Field2d

    def __init__(self) -> None:
        self.camera = PhotonCamera("forward_camera")
        self.has_targets = False
        self.targets = []
        self.last_timestamp = 0

    def setup(self) -> None:
        self.field_pos_obj = self.field.getObject("vision_pose")

    @staticmethod
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

    @staticmethod
    def point_cloud_centroid(
        points: list[tuple[float, float]]
    ) -> tuple[float, float, float, float]:
        l = len(points)
        accx = accy = 0
        for (x, y) in points:
            accx += x
            accy += y
        mx = accx / l
        my = accy / l
        accx = accy = 0
        for (x, y) in points:
            dx = x - mx
            dy = y - my
            accx += dx * dx
            accy += dy * dy
        return (
            mx,
            my,
            Vision.STD_DEV_CONSTANT / l + sqrt(accx / l),
            Vision.STD_DEV_CONSTANT / l + sqrt(accy / l),
        )

    def execute(self) -> None:
        results = self.camera.getLatestResult()
        self.has_targets = results.hasTargets()
        timestamp = results.getTimestamp()

        if not self.has_targets:
            return
        if timestamp == self.last_timestamp and wpilib.RobotBase.isReal():
            return

        self.last_timestamp = timestamp
        self.targets = results.getTargets()
        chassis_rotation = self.chassis.get_pose_at(timestamp).rotation()
        estimated_pose = Pose2d()
        std_dev_x = std_dev_y = 1000000
        if len(self.targets) == 1:
            # Use the only possibility
            estimated_pose = Vision.estimate_pos_from_apriltag(
                Vision.FORWARD_CAMERA_TRANSFORM, self.targets[0]
            )
            std_dev_x = std_dev_y = (
                2 * Vision.STD_DEV_CONSTANT
            )  # Don't trust a single reading much
        else:
            # Locate using position resection
            tag_positions = []
            tag_neg_unit_transls = []
            for t in self.targets:
                tag_id = t.getFiducialId()
                tag_pose = Vision.FIELD_LAYOUT.getTagPose(tag_id)
                if tag_pose is None:
                    print("AprilTag with ID {} not found")
                    return
                tag_positions.append((tag_pose.x, tag_pose.y))
                world_transl = (
                    t.getBestCameraToTarget()
                    .translation()
                    .toTranslation2d()
                    .rotateBy(-tag_pose.rotation())
                )
                world_cam_to_robot = (
                    Vision.FORWARD_CAMERA_TRANSFORM.translation()
                    .toTranslation2d()
                    .rotateBy(chassis_rotation)
                )
                # Get estimated pose of chassis and not camera
                world_transl -= world_cam_to_robot
                hypot = world_transl.norm()
                if hypot < Vision.ZERO_DIVISION_THRESHOLD:
                    print("AprilTag with ID {} apears to be unreasonably close")
                    return
                tag_neg_unit_transls.append(
                    (-world_transl.X() / hypot, -world_transl.Y() / hypot)
                )
            l = len(tag_positions)
            intersections = []
            for i in range(l):
                for j in range(i, l):
                    ix = tag_positions[i][0]
                    iy = tag_positions[i][1]
                    jx = tag_positions[j][0]
                    idx = tag_neg_unit_transls[i][0]
                    idy = tag_neg_unit_transls[i][1]
                    jdx = tag_neg_unit_transls[j][0]
                    if abs(idx - jdx) < Vision.ZERO_DIVISION_THRESHOLD:
                        # TODO: Remove:
                        print("Skipping parallel vectors")
                        continue  # Unit vectors seem parallel
                    t = (jx - ix) / (idx - jdx)
                    if t < 0:
                        # TODO: Remove:
                        print("Skipping negative t")
                        continue  # Can't be behind apriltags
                    x = ix + idx * t
                    y = iy + idy * t
                    intersections.append((x, y))
            l = len(intersections)
            if (
                l == 0
            ):  # if no intersections, take centroid of reverse-projected apriltag position to camera
                positions = [
                    (p.X(), p.Y())
                    for p in (
                        Vision.estimate_pos_from_apriltag(
                            Vision.FORWARD_CAMERA_TRANSFORM, t
                        )
                        for t in self.targets
                    )
                ]
                mx, my, std_dev_x, std_dev_y = Vision.point_cloud_centroid(positions)
                estimated_pose = Pose2d(mx, my, 0)
            else:
                mx, my, std_dev_x, std_dev_y = Vision.point_cloud_centroid(
                    intersections
                )
                estimated_pose = Pose2d(mx, my, 0)

        # Overwrite rotation given by vision with what gyro gives
        estimated_pose = Pose2d(estimated_pose.translation(), chassis_rotation)

        self.field_pos_obj.setPose(estimated_pose)

        self.chassis.estimator.addVisionMeasurement(
            estimated_pose, timestamp, [std_dev_x, std_dev_y, 1.0]
        )
