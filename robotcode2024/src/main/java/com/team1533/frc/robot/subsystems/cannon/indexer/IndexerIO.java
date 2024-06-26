package com.team1533.frc.robot.subsystems.cannon.indexer;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public boolean motorConnected = true;

        public double inputVolts = 0.0;
        public double motorCurrent = 0.0;
        public double appliedVolts = 0.0;

        public double linearVel = 0.0;

        public double tempCelcius = 0.0;

        public boolean sensor = false;

    }

    default void updateInputs(IndexerIOInputs inputs) {
    }

    /** Run motors at volts */
    default void runVolts(double volts) {
    }

    default void runSpeed(double speed) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode idleMode) {
    }

    /** Stops motors */
    default void stop() {
    }
}