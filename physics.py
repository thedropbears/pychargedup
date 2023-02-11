from __future__ import annotations

import math
import typing

import ctre
import numpy as np
import wpilib
from wpilib.simulation import (
    SingleJointedArmSim,
    ElevatorSim,
    SimDeviceSim,
    DutyCycleEncoderSim,
    SolenoidSim,
)
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor

from components.chassis import SwerveModule
from components import arm
from utilities.ctre import FALCON_CPR, VERSA_ENCODER_CPR
from ids import PcmChannels, SparkMaxIds

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

        # Create arm simulation
        arm_motors_sim = DCMotor.NEO(2)
        arm_len = (arm.MIN_EXTENSION + arm.MAX_EXTENSION) / 2
        arm_mass = 5
        moi = SingleJointedArmSim.estimateMOI(arm_len, arm_mass)
        self.arm_sim = SingleJointedArmSim(
            arm_motors_sim,
            robot.arm.ROTATE_GEAR_RATIO,
            moi,
            arm_len,
            -arm.MAX_ANGLE - math.radians(10),
            -arm.MIN_ANGLE,
            arm_mass,
            simulateGravity=True,
            measurementStdDevs=[math.radians(0.01)],
        )
        extension_motors_sim = DCMotor.NEO550(1)
        self.extension_sim = ElevatorSim(
            extension_motors_sim,
            robot.arm.EXTEND_GEAR_RATIO,
            4,
            robot.arm.SPOOL_DIAMETER / 2,
            arm.MIN_EXTENSION,
            arm.MAX_EXTENSION,
            simulateGravity=False,
            measurementStdDevs=[0.001],
        )
        self.extension_sim.setState(np.array([[arm.MIN_EXTENSION], [0]]))

        # Get arm objects
        self.arm_abs_encoder = DutyCycleEncoderSim(robot.arm.absolute_encoder)
        self.arm_brake = SolenoidSim(
            wpilib.PneumaticsModuleType.CTREPCM, PcmChannels.arm_brake
        )

        self.arm_motor = SimDeviceSim("SPARK MAX ", SparkMaxIds.arm_rotation_main)
        self.arm_motor_pos = self.arm_motor.getDouble("Position")
        self.arm_motor_vel = self.arm_motor.getDouble("Velocity")
        self.arm_motor_output = self.arm_motor.getDouble("Applied Output")

        self.arm_extension = SimDeviceSim("SPARK MAX ", SparkMaxIds.arm_extension)
        self.arm_extension_pos = self.arm_extension.getDouble("Position")
        self.arm_extension_vel = self.arm_extension.getDouble("Velocity")
        self.arm_extension_output = self.arm_extension.getDouble("Applied Output")

        self.imu = SimDeviceSim("navX-Sensor", 4)
        self.imu_yaw = self.imu.getDouble("Yaw")

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Update rotation sim
        self.arm_sim.setInputVoltage(-self.arm_motor_output.get())
        if self.arm_brake.getOutput():
            self.arm_sim.update(tm_diff)
        self.arm_abs_encoder.setDistance(-self.arm_sim.getAngle())
        self.arm_motor_vel.set(-self.arm_sim.getVelocity())
        # Update extension sim
        self.extension_sim.setInputVoltage(self.arm_extension_output.get())
        self.extension_sim.update(tm_diff)
        self.arm_extension_pos.set(self.extension_sim.getPosition())
        self.arm_extension_vel.set(self.extension_sim.getVelocity())

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds(
            *(module.get() for module in self.swerve_modules)
        )

        self.imu_yaw.set(self.imu_yaw.get() - math.degrees(speeds.omega * tm_diff))

        self.physics_controller.drive(speeds, tm_diff)
