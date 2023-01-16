from __future__ import annotations

import math
import typing

import ctre
import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import SwerveDrive4Kinematics

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

        self.imu = wpilib.simulation.SimDeviceSim("navX-Sensor", 4)
        self.imu_yaw = self.imu.getDouble("Yaw")

    def update_sim(self, now: float, tm_diff: float) -> None:
        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds(
            *(module.get() for module in self.swerve_modules)
        )

        self.imu_yaw.set(self.imu_yaw.get() - math.degrees(speeds.omega * tm_diff))

        self.physics_controller.drive(speeds, tm_diff)
