package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.elevator.Elevator.IntakePosition;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeMotorConnected = true;

        public double intakeInputVolts = 0.0;
        public double intakeMotorCurrent = 0.0;
        public double intakeAppliedVolts = 0.0;

        public double intakeLinearVel = 0.0;

        public double intakeTempCelcius = 0.0;

    }

    default void updateInputs(IntakeIOInputs inputs) {
    }

    /** Run motors at volts */
    default void runIntakeVolts(double intakeVolts) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode intakeIdleMode) {
    }

    default boolean getIntakeSensor() {
        return false;
    }

    default boolean getBlocked(DigitalInput proxInput) {
        return false;
    }

    /** Stops motors */
    default void stop() {
    }
}