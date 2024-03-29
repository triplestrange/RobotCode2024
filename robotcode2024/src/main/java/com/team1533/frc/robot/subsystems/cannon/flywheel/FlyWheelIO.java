package com.team1533.frc.robot.subsystems.cannon.flywheel;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

public interface FlyWheelIO {

    @AutoLog
    public static class FlyWheelIOInputs {
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;

        public double leftInputVolts = 0.0;
        public double leftMotorCurrent = 0.0;
        public double leftAppliedVolts = 0.0;
        public double rightInputVolts = 0.0;
        public double rightMotorCurrent = 0.0;
        public double rightAppliedVolts = 0.0;

        public double leftVel = 0.0;
        public double rightVel = 0.0;

        public double leftTempCelcius = 0.0;
        public double rightTempCelcius = 0.0;

        public double leftSetpoint = 0.0;
        public double rightSetpoint = 0.0;
    }

    default void updateInputs(FlyWheelIOInputs inputs) {
    }

    default void setLeftSpeed(double RPM) {
    }

    default void setRightSpeed(double RPM) {
    }

    default void runSpeed(double RPM) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode idleMode) {
    }

    /** Stops motors */
    default void stop() {
    }
}