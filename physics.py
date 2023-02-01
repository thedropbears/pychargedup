from __future__ import annotations

import math
import typing

import ctre
import wpilib
from wpilib.simulation import (
    SingleJointedArmSim,
    ElevatorSim,
    SimDeviceSim,
    DutyCycleEncoderSim,
)
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor
from wpimath.trajectory import TrapezoidProfile

from components.chassis import SwerveModule
from utilities.ctre import FALCON_CPR, VERSA_ENCODER_CPR

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(self, motor: ctre.TalonFX, kV: float, rev_per_unit: float) -> None:
        self.sim_collection = motor.getSimCollection()
        self.kV = kV  # volt seconds per unit
        self.rev_per_unit = rev_per_unit

    def update(self, dt: float) -> None:
        voltage = self.sim_collection.getMotorOutputLeadVoltage()
        velocity = voltage / self.kV  # units per second
        velocity_cps = velocity * self.rev_per_unit * FALCON_CPR
        self.sim_collection.setIntegratedSensorVelocity(int(velocity_cps / 10))
        self.sim_collection.addIntegratedSensorPosition(int(velocity_cps * dt))


class SimpleTalonSRXMotorSim:
    def __init__(self, motor: ctre.TalonSRX, kV: float, rev_per_unit: float) -> None:
        self.sim_collection = motor.getSimCollection()
        self.kV = kV  # volt seconds per unit
        self.rev_per_unit = rev_per_unit

    def update(self, dt: float) -> None:
        voltage = self.sim_collection.getMotorOutputLeadVoltage()
        velocity = voltage / self.kV  # units per second
        velocity_cps = velocity * self.rev_per_unit * VERSA_ENCODER_CPR
        self.sim_collection.setQuadratureVelocity(int(velocity_cps / 10))
        self.sim_collection.addQuadraturePosition(int(velocity_cps * dt))


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: list[SwerveModule] = robot.chassis.modules

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive, module.drive_ff.kV, 1 / module.DRIVE_MOTOR_REV_TO_METRES
            )
            for module in robot.chassis.modules
        ]
        self.steer = [
            SimpleTalonFXMotorSim(
                module.steer,
                kV=1,  # TODO: get from sysid logs
                rev_per_unit=1 / module.STEER_MOTOR_REV_TO_RAD,
            )
            for module in robot.chassis.modules
        ]

        self.arm = robot.arm
        # Create arm simulation
        arm_motors_sim = DCMotor.NEO(2)
        arm_len = (robot.arm.MIN_EXTENSION + robot.arm.MAX_EXTENSION) / 2
        arm_mass = 5
        moi = SingleJointedArmSim.estimateMOI(arm_len, arm_mass)
        self.arm_sim = SingleJointedArmSim(
            arm_motors_sim,
            robot.arm.ROTATE_GEAR_RATIO,
            moi,
            arm_len,
            -robot.arm.MAX_ANGLE,
            -robot.arm.MIN_ANGLE,
            arm_mass,
            True,
            [math.radians(0.01)],
        )
        extension_motors_sim = DCMotor.NEO550(1)
        self.extension_sim = ElevatorSim(
            extension_motors_sim,
            robot.arm.EXTEND_GEAR_RATIO,
            4,
            robot.arm.SPOOL_DIAMETER / 2,
            0,
            robot.arm.MAX_EXTENSION - robot.arm.MIN_EXTENSION,
            True,
            [0.001],
        )
        self.arm_abs_encoder = DutyCycleEncoderSim(robot.arm.absolute_encoder)

        # Create arm display
        self.arm_mech2d = wpilib.Mechanism2d(5, 3)
        arm_pivot = self.arm_mech2d.getRoot("ArmPivot", 2.5, robot.arm.HEIGHT)
        arm_pivot.appendLigament("ArmTower", robot.arm.HEIGHT - 0.05, -90)
        self.arm_ligament = arm_pivot.appendLigament(
            "Arm", robot.arm.MIN_EXTENSION, self.arm_sim.getAngleDegrees()
        )
        self.arm_goal_ligament = arm_pivot.appendLigament(
            "Arm_goal",
            robot.arm.MIN_EXTENSION,
            self.arm_sim.getAngleDegrees(),
            3,
            wpilib.Color8Bit(235 // 2, 137 // 2, 52 // 2),
        )
        self.arm_extend_ligament = self.arm_ligament.appendLigament(
            "ArmExtend",
            0.5,
            self.arm_sim.getAngleDegrees(),
            8,
            wpilib.Color8Bit(137, 235, 52),
        )
        self.arm_extend_goal_ligament = self.arm_ligament.appendLigament(
            "ArmExtend_goal",
            0.1,
            self.arm_sim.getAngleDegrees(),
            4,
            wpilib.Color8Bit(137 // 2, 235 // 2, 52 // 2),
        )
        wpilib.SmartDashboard.putData("Arm sim", self.arm_mech2d)

        self.imu = SimDeviceSim("navX-Sensor", 4)
        self.imu_yaw = self.imu.getDouble("Yaw")

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Update rotation sim
        self.arm_sim.setInputVoltage(
            -self.arm.rotation_motor.get() * wpilib.RobotController.getBatteryVoltage()
        )
        if self.arm.brake_solenoid.get():
            self.arm_sim.update(tm_diff)
        self.arm_abs_encoder.set(-self.arm_sim.getAngle() / math.tau)
        self.arm.relative_encoder.set_velocity(-self.arm_sim.getVelocity())
        # Update extension sim
        self.extension_sim.setInputVoltage(
            self.arm.extension_motor.get() * wpilib.RobotController.getBatteryVoltage()
        )
        self.extension_sim.update(tm_diff)
        self.arm.extension_encoder.setPosition(self.extension_sim.getPosition())

        # Update display
        self.arm_ligament.setAngle(self.arm_sim.getAngleDegrees())
        s: TrapezoidProfile.State = self.arm.rotation_controller.getSetpoint()
        self.arm_goal_ligament.setAngle(-math.degrees(s.position))
        self.arm_extend_ligament.setLength(self.arm.extension_encoder.getPosition())
        s = self.arm.extension_controller.getSetpoint()
        self.arm_extend_goal_ligament.setLength(s.position - self.arm.MIN_EXTENSION)

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds(
            *(module.get() for module in self.swerve_modules)
        )

        self.imu_yaw.set(self.imu_yaw.get() - math.degrees(speeds.omega * tm_diff))

        self.physics_controller.drive(speeds, tm_diff)
