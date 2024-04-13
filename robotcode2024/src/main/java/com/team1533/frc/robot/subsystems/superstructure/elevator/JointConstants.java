package com.team1533.frc.robot.subsystems.superstructure.elevator;

public class JointConstants {
    public final static double kMaxAngularSpeedMetersPerSecond = 360;
    public final static double kMaxAngularAccelerationMetersPerSecondSquared = 1200;

    public final static double intakeAbsOffset = -78.33 + 10.7 - 22;

    public final static double kP = 0.035 * .5;
    public final static double kI = 0;
    public final static double kD = 0;

    public final static double maxAngle = 0;
    public final static double minAngle = -145;
    public final static double safeZone = 0;
}
