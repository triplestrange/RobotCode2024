package com.team1533.frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;
    public static final double kDriveEncoderCPR = 6.75;
    public static final double kSteerEncoderCPR = (150.0d / 7);

    // adjust for calibration
    // 2/25/21 - 0.12584
    public static final double offsetRatio = 1;
    public static final double kWheelDiameterMeters = .1016 * offsetRatio;
    public static final double kDriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kDriveEncoderCPR;
    public static final double kSteerEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kSteerEncoderCPR;
    public static final int FL_ENCODER = 1;
    public static final int FR_ENCODER = 0;
    public static final int BL_ENCODER = 3;
    public static final int BR_ENCODER = 2;
    public final static double FL_ENC_OFFSET = 56 - 4; // 183
    public final static double FR_ENC_OFFSET = 13 - 4; // 179 141
    public final static double BL_ENC_OFFSET = Units.radiansToDegrees(2.642752) - 4; // 221
    public final static double BR_ENC_OFFSET = 322 + 4 - 4; // 241

    public final static boolean driveEnableCurrentLimit = true;
    public final static int driveContinuousCurrentLimit = 35;
    public final static int drivePeakCurrentLimit = 40;
    public final static int drivePeakCurrentDuration = 1;
}
