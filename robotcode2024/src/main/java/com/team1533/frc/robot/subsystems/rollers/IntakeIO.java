package com.team1533.frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public boolean motorConnected = true;

        public double inputVolts = 0.0;
        public double motorCurrent = 0.0;
        public double appliedVolts = 0.0;

        public double linearVel = 0.0;

        public double tempCelcius = 0.0;

        public boolean sensor = false;

    }

    default void updateInputs(IntakeIOInputs inputs) {
    }

    /** Run motors at volts */
    default void runVolts(double volts) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode idleMode) {
    }

    default boolean getBlocked(DigitalInput proxInput) {
        return false;
    }

    /** Stops motors */
    default void stop() {
    }
}