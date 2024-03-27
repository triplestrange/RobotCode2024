package frc.robot.subsystems.cannon.climb;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.cannon.climb.Climb.ClimbPosition;
import frc.robot.subsystems.intake.elevator.Elevator.IntakePosition;

public interface ClimbIO {

    @AutoLog
    public static class ClimbIOInputs {
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

    default void updateInputs(ClimbIOInputs inputs) {
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