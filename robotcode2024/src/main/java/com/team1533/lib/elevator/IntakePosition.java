package com.team1533.lib.elevator;

public class IntakePosition {
    private double elevPos;
    private double intakeAng;

    public IntakePosition(double elevPos, double intakeAng) {
        this.elevPos = elevPos;
        this.intakeAng = intakeAng;
    }

    // height from bottom of carriage
    public double getHeight() {
        return elevPos;
    }

    public double getAngle() {
        return intakeAng;
    }
}