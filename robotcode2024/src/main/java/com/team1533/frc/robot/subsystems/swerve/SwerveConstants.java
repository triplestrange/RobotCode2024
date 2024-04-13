package com.team1533.frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {
    // kraken = 5.21208, neo = 4.42, vortex = 5.88264
    public static final double kTheoreticalMaxSpeedMetersPerSecond = 4.7244;
    public static final double kMaxSpeedMetersPerSecond = 3.77;
    public static final double kMaxAngularVelocityRadiansPerSecond = 2 * Math.PI;

    public static final double kScaleTranslationInputs = 1;

    public static final boolean kGyroReversed = true;

    // encoder's aren't reversed
    public static final boolean frontLeftSteerEncoderReversed = false;
    public static final boolean backLeftSteerEncoderReversed = false;
    public static final boolean frontRightSteerEncoderReversed = false;
    public static final boolean backRightSteerEncoderReversed = false;

    // Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = 0.52705;
    // Distance between front and back wheels on robot in meters
    public static final double kWheelBase = 0.52705;

    public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);

    // kinematics constructor with module positions as arguments
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final class RotationConfigs {

        public static final double kSwerveHeadingControllerErrorTolerance = 1.5;

        public static final double kSnapSwerveHeadingKp = 0.1;
        public static final double kSnapSwerveHeadingKi = 0.17;
        public static final double kSnapSwerveHeadingKd = 0.005;

        public static final double kMaintainSwerveHeadingKpHighVelocity = 0.1;
        public static final double kMaintainSwerveHeadingKiHighVelocity = 0.1;
        public static final double kMaintainSwerveHeadingKdHighVelocity = 0.0025;

    }

    public static final class AutoAlignConstants {
        public static final double kAutoALignControllerErrorTolerance = 0.0762;
        public static final double autoAlignMaxSpeedMetersPerSecond = 1;
        public static final double autoAlignRotationalMaxSpeedMetersPerSecond = 1;
    }

}
