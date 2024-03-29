package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.elevator.Elevator.IntakePosition;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public boolean motorConnected = true;

        public double inputVolts = 0.0;
        public double motorCurrent = 0.0;
        public double appliedVolts = 0.0;

        public double linearVel = 0.0;

        public double tempCelcius = 0.0;

    }

    default void updateInputs(IntakeIOInputs inputs) {
    }

    /** Run motors at volts */
    default void runVolts(double volts) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode idleMode) {
    }

    default boolean getSensor() {
        return false;
    }

    default boolean getBlocked(DigitalInput proxInput) {
        return false;
    }

    /** Stops motors */
    default void stop() {
    }
}