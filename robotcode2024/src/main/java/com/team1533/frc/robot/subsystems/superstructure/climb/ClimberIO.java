package com.team1533.frc.robot.subsystems.superstructure.climb;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.CANSparkBase.IdleMode;
import com.team1533.frc.robot.subsystems.superstructure.climb.Climber.ClimbPosition;

public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;

        public double leftInputVolts = 0.0;
        public double leftMotorCurrent = 0.0;
        public double leftAppliedVolts = 0.0;

        public double leftPosMeters = 0.0;
        public double leftVelMetersPerSecond = 0.0;

        public double rightInputVolts = 0.0;
        public double rightMotorCurrent = 0.0;
        public double rightAppliedVolts = 0.0;

        public double rightPosMeters = 0.0;
        public double rightVelMetersPerSecond = 0.0;

        public double leftTempCelcius = 0.0;
        public double rightTempCelcius = 0.0;

        public double leftSpeed = 0.0;
        public double rightSpeed = 0.0;
    }

    default void updateInputs(ClimberIOInputs inputs) {
    }

    /** Run to setpoint IntakePosition in meters and degrees */
    default void runClimbSetpoints(ClimbPosition position) {
    }

    /** Run motors at volts */
    default void runLeftVolts(double leftVolts) {
    }

    default void runLeftSpeed(double leftSpeed) {
    }

    default void runRightVolts(double rightVolts) {
    }

    default void runRightSpeed(double rightSpeed) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode idleMode) {
    }

    /** Sets position of internal encoder in inches */
    default void setLeftClimb(double setpoint) {
    }

    default void setRightClimb(double setpoint) {
    }

    /** Stops motors */
    default void stop() {
    }
}