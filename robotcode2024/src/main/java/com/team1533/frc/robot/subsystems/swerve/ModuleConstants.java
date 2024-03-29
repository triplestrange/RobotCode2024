package com.team1533.frc.robot.subsystems.swerve;

import com.team1533.frc.robot.Constants;
import com.team1533.lib.swerve.ModuleConfig;

import edu.wpi.first.math.util.Units;

public class ModuleConstants {
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

        public final static boolean driveEnableCurrentLimit = true;
        public final static int driveContinuousCurrentLimit = 35;
        public final static int drivePeakCurrentLimit = 40;
        public final static int drivePeakCurrentDuration = 1;

        public final static ModuleConfig FL = new ModuleConfig(Constants.CAN.FL_DRIVE, Constants.CAN.FL_STEER, 1,
                        false, 56 - 4);
        public final static ModuleConfig FR = new ModuleConfig(Constants.CAN.FR_DRIVE, Constants.CAN.FR_STEER, 0,
                        false, 13 - 4);
        public final static ModuleConfig BL = new ModuleConfig(Constants.CAN.BL_DRIVE, Constants.CAN.BL_STEER, 3,
                        false, Units.radiansToDegrees(2.642752) - 4);
        public final static ModuleConfig BR = new ModuleConfig(Constants.CAN.BR_DRIVE, Constants.CAN.BR_STEER, 2,
                        false, 326 - 4);
}
