package com.team1533.frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

    public final static double kMaxSpeedInchesPerSecond = 24.5;
    public final static double kMaxAccelerationInchesPerSecondSquared = 100;

    public final static double maxHeightInches = 10.75;
    public final static double minHeightInches = 0;
    public final static double safeZoneInches = 0;

    public final static double elevPosConv = maxHeightInches  / 47.358;
    public final static double elevSimPosConv = 15;
    public final static double elevCarraigeKG = Units.lbsToKilograms(13.667);
    public final static double elevDrumRadiusMeters = Units.inchesToMeters(1.25);

    public final static double kP = 0.4;
    public final static double kI = 0;
    public final static double kD = 0;

    public final static double kMaxOutput = 1;
    public final static double kMinOutput = -1;
    public final static double maxRPM = 0;
    public final static double allowedErr = 0;
    public final static double maxVel = 1;
    public final static double minVel = 1;
    public final static double maxAcc = 1;
}
