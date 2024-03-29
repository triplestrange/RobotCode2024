package com.team1533.frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.CANSparkBase.IdleMode;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public boolean leftMotorConnected = true;
        public boolean absoluteEncoderConnected = true;

        public double leftInputVolts = 0.0;
        public double leftInputSpeed = 0.0;
        public double leftMotorCurrent = 0.0;
        public double leftAppliedVolts = 0.0;

        public double posDeg = 0.0;
        public double jointVelDegPerSecond = 0.0;
        public double leftVelDegPerSescond = 0.0;

        public double leftTempCelcius = 0.0;

        public boolean rightMotorConnected = true;

        public double rightInputVolts = 0.0;
        public double rightInputSpeed = 0.0;
        public double rightMotorCurrent = 0.0;
        public double rightAppliedVolts = 0.0;

        public double rightPosDeg = 0.0;
        public double rightVelDegPerSecond = 0.0;

        public double rightTempCelcius = 0.0;

    }

    default void updateInputs(ArmIOInputs inputs) {
    }

    default void runVolts(double volts) {
    }

    default void runPower(double power) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode rightIdleMode, IdleMode leftIdleMode) {
    }

    /** Stops motors */
    default void stop() {
    }
}