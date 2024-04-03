package com.team1533.frc.robot.subsystems.cannon.flywheel;

import edu.wpi.first.math.util.Units;

public class FlyWheelConstants {

    // flywheel motor constants

    public final static double kMaxflyWheelSpeedMetersPerSecond = 0;
    public final static double kMaxflyWheelAccelerationMetersPerSecondSquared = 0;

    public final static double flyWheelkP = 0.0001;
    public final static double flyWheelkI = 0;
    public final static double flyWheelkD = 0;
    public final static double flyWheelkIz = 0;
    public final static double flyWheelkFF = 1. / 4950;

    public final static double flyWheelkMaxOutput = 1;
    public final static double flyWheelkMinOutput = -1;
    public final static double flyWheelmaxRPM = 5000;
    public final static double flyWheelallowedErr = 0;
    public final static double flyWheelmaxVel = 5767;
    public final static double flyWheelminVel = -5767;
    public final static double flyWheelmaxAcc = 5000;

    public final static double rotationalSpeed = 3000;

    public final static double flywheelVelocityConv = 2 * Units.inchesToMeters(3);

}
