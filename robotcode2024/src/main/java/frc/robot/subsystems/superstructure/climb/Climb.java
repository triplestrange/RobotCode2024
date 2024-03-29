package frc.robot.subsystems.superstructure.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb {

    public double leftPower;
    public double rightPower;

    private static Climb instance;

    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public static Climb getInstance() {
        return instance;
    }

    public static Climb initialize(ClimbIO io) {
        if (instance == null) {
            instance = new Climb(io);
        }
        return instance;
    }

    public Climb(ClimbIO climbIO) {
        super();

        io = climbIO;
        io.updateInputs(inputs);

    }

    public double getLWinchPos() {
        return inputs.leftPosMeters;
    }

    public double getRWinchPos() {
        return inputs.rightPosMeters;
    }

    public void moveClimb(double motorLWinchPower, double motorRWinchPower) {

        if (Math.abs(motorLWinchPower) < 0.05) {
            io.runLeftSpeed(motorLWinchPower);
        } else {
            leftPower = motorLWinchPower;
            io.runLeftSpeed(leftPower);
        }

        if (Math.abs(motorRWinchPower) < 0.05) {
            io.runRightSpeed(motorRWinchPower);
        } else {
            rightPower = motorRWinchPower;
            io.runRightSpeed(rightPower);
        }
    }

    public static class ClimbPosition {
        private double lWinchPos;
        private double rWinchPos;

        public ClimbPosition(double lWinchPos, double rWinchPos) {
            this.lWinchPos = lWinchPos;
            this.rWinchPos = rWinchPos;
        }

        // height from bottom of carriage
        public double getLeftClimb() {
            return lWinchPos;
        }

        public double getRightClimb() {
            return rWinchPos;
        }
    }

    public void periodic() {
    }

    public void updateSmartDashBoard() {

    }
}
