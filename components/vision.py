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

    FIELD_LAYOUT = robotpy_apriltag.AprilTagFieldLayout(
        wpilib.getDeployDirectory() + "/test_field_layout.json"
    )
    FORWARD_CAMERA_TRANSFORM = Transform3d(
        Translation3d(-0.35, 0.01, 0.11), Rotation3d.fromDegrees(0, 0, 175)
    )
    STD_DEV_CONSTANT = 5.0
    ZERO_DIVISION_THRESHOLD = 1e-6
    POSE_AMBIGUITY_FACTOR = 4.0
    POSE_AMBIGUITY_THRESHOLD = 0.25

    field: wpilib.Field2d
    use_resection = tunable(False)

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
        f = 1.0 / len(points)
        accx = accy = 0
        for (x, y) in points:
            accx += x
            accy += y
        mx = accx * f
        my = accy * f
        accx = accy = 0
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

    @staticmethod
    def weighted_point_cloud_centroid(
        points: list[tuple[float, float, float]]
    ) -> tuple[float, float, float, float]:
        accx = accy = accw = 0
        for (x, y, w) in points:
            accx += x * w
            accy += y * w
            accw += w
        f = 1.0 / accw
        mx = accx * f
        my = accy * f
        accx = accy = accw = 0
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
        chassis_rotation = self.chassis.get_pose().rotation()
        estimated_pose = Pose2d()
        std_dev_x = std_dev_y = math.inf
        if self.use_resection:
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
                        print(f"AprilTag with ID {tag_id} not found")
                        return
                    tag_positions.append((tag_pose.x, tag_pose.y))
                    world_transl = (
                        t.getBestCameraToTarget()
                        .translation()
                        .toTranslation2d()
                        .rotateBy(-tag_pose.toPose2d().rotation())
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
                        print(f"AprilTag with ID {tag_id} apears to be unreasonably close")
                        return
                    tag_neg_unit_transls.append(
                        (-world_transl.X() / hypot, -world_transl.Y() / hypot)
                    )
                l = len(tag_positions)
                intersections = []
                for i in range(l - 1):
                    for j in range(i + 1, l):
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
        else:
            estimated_poses = [Vision.estimate_pos_from_apriltag(Vision.FORWARD_CAMERA_TRANSFORM, t) for t in self.targets]
            weights = [max(1.0 - Vision.POSE_AMBIGUITY_FACTOR * t.getPoseAmbiguity(), 0) for t in self.targets]
            if any(w < Vision.ZERO_DIVISION_THRESHOLD for w in weights): return
            points = [(p.x, p.y, w) for (p, w) in zip(estimated_poses, weights)]
            mx, my, std_dev_x, std_dev_y = Vision.weighted_point_cloud_centroid(
                points
            )
            rotation_unit_vectors = [(p.rotation().cos(), p.rotation().sin()) for p in estimated_poses]
            accx = accy = 0
            for (x, y), w in zip(rotation_unit_vectors, weights):
                accx += x * w
                accy += y * w
            f = 1 / math.hypot(accx, accy)
            estimated_pose = Pose2d(mx, my, Rotation2d(accx * f, accy * f))


        # Overwrite rotation given by vision with what gyro gives
        # estimated_pose = Pose2d(estimated_pose.translation(), chassis_rotation)

        self.field_pos_obj.setPose(estimated_pose)

        self.chassis.estimator.addVisionMeasurement(
            estimated_pose, timestamp, [std_dev_x, std_dev_y, 3.0]
        )
