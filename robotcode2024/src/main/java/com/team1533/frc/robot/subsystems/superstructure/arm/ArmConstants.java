package com.team1533.frc.robot.subsystems.superstructure.arm;

public class ArmConstants {
    public final static double kMaxAngularPivotSpeedDegreesPerSecond = 300; // theoretical max speed 476.784;
    public final static double kMaxAngularPivotAccelerationDegreesPerSecondSquared = 300;

    public final static double maxAngle = 0;
    public final static double minAngle = -48;
    public final static double safeZone = 1;

    public final static double pivotkP = 0.03;
    public final static double pivotkI = 0.045;
    public final static double pivotkD = 0.00025;

    public final static double pivotAbsOffset = -105;
}
