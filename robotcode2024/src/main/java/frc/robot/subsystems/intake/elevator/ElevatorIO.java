package frc.robot.subsystems.intake.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double winchMotorAppliedVolts = 0.0;
        public double winchMotorCurrent = 0.0;
        public double winchAppliedVolts = 0.0;

        public double elevatorPosMeters = 0.0;
        public double elevatorVelMetersPerSecond = 0.0;

        public double jointMotorAppliedVolts = 0.0;
        public double jointMotorCurrent = 0.0;
        public double jointAppliedVolts = 0.0;

        public double jointPosDeg = 0.0;
        public double joinVelDeg = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void moveElevator(double speed) {
    }

    public default void setVoltage(double volts) {
    }

}