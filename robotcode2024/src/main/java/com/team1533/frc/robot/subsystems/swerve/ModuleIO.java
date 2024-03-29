package com.team1533.frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveMotorConnected = true;
        public boolean turnMotorConnected = true;
        public boolean absMotorConnected = true;
        public boolean hasCurrentControl = false;

        public double drivePositionRads = 0.0;
        public double driveVelocityRadsPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveTorqueCurrentAmps = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public Rotation2d turnVelocityPerSec = new Rotation2d();
        public double turnSpeed = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;
        public double turnTorqueCurrentAmps = 0.0;

        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Run drive motor at volts */
    default void runDriveVolts(double volts) {
    }

    /** Run turn motor at volts */
    default void runTurnVolts(double volts) {
    }

    /** Run characterization input (amps or volts) into drive motor */
    default void runCharacterization(double input) {
    }

    /** Run to drive velocity setpoint with feedforward */
    default void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    }

    /** Run to turn position setpoint */
    default void runTurnPositionSetpoint(double angleRads) {
    }

    default void setDesiredState(SwerveModuleState state) {
    }

    default void setDesiredState(SwerveModuleState state, boolean forceAngle) {
    }

    /** Configure drive PID */
    default void setDrivePID(double kP, double kI, double kD) {
    }

    /** Configure turn PID */
    default void setTurnPID(double kP, double kI, double kD) {
    }

    default void resetEncoders() {
    }

    /** Enable or disable brake mode on the drive motor. */
    default void setDriveBrakeMode(NeutralModeValue neutralModeValue) {
    }

    /** Enable or disable brake mode on the turn motor. */
    default void setTurnBrakeMode(IdleMode idleMode) {
    }

    /** Disable output to all motors */
    default void stop() {
    }
}