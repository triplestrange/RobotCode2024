package com.team1533.frc.robot.subsystems.superstructure.climb;

public class Climber {

    public double leftPower;
    public double rightPower;

    private static Climber instance;

    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public static Climber getInstance() {
        return instance;
    }

    public static Climber initialize(ClimberIO io) {
        if (instance == null) {
            instance = new Climber(io);
        }
        return instance;
    }

    public Climber(ClimberIO climbIO) {
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
            io.runLeftSpeed(0);
        } else {
            leftPower = motorLWinchPower;
            io.runLeftSpeed(leftPower);
        }

        if (Math.abs(motorRWinchPower) < 0.05) {
            io.runRightSpeed(0);
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
