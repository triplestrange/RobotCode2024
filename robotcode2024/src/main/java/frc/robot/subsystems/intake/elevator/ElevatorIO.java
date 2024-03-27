package frc.robot.subsystems.intake.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.elevator.Elevator.IntakePosition;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public boolean elevMotorConnected = true;
        public boolean jointAbsoluteEncoderConnected = true;

        public double winchInputVolts = 0.0;
        public double winchMotorCurrent = 0.0;
        public double winchAppliedVolts = 0.0;

        public double elevatorPosMeters = 0.0;
        public double elevatorVelMetersPerSecond = 0.0;

        public double jointInputVolts = 0.0;
        public double jointInputSpeed = 0.0;
        public double jointMotorCurrent = 0.0;
        public double jointAppliedVolts = 0.0;

        public double jointPosDeg = 0.0;
        public double joinVelDegPerSecond = 0.0;

        public double winchTempCelcius = 0.0;
        public double jointTempCelcius = 0.0;

    }

    default void updateInputs(ElevatorIOInputs inputs) {
    }

    /** Run to setpoint IntakePosition in meters and degrees */
    default void runHeightSetpoint(double height) {
    }

    /** Run motors at volts */
    default void runWinchVolts(double elevVolts) {
    }

    default void runJointVolts(double intakeVolts) {
    }

    default void runJointPower(double jointPower) {
    }

    /** Set brake mode enabled */
    default void setIdleMode(IdleMode elevIdleMode, IdleMode jointIdleMode) {
    }

    /** Sets position of internal encoder in inches */
    default void setElevPosition(double height) {
    }

    default void setJointPosition(double angle) {
    }

    /** Stops motors */
    default void stop() {
    }
}