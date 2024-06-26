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
        public static final double offsetRatio = 1.045;
        public static final double kWheelDiameterMeters = .1016 * 1./offsetRatio;
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
        // PID coefficients
        public final static double tkP = 6; // 0.5
        public final static double tkMaxOutput = 1;
        public final static double tkMinOutput = -1;

        public final static double dkP = 2;
        public final static double dkI = 1;
        public final static double dffkS = 0;
        public final static double dffkV = 12 / SwerveConstants.kMaxSpeedMetersPerSecond;

        public final static ModuleConfig FL = new ModuleConfig(Constants.CAN.FL_DRIVE, Constants.CAN.FL_STEER, 1,
                        false, 54); // 52
        public final static ModuleConfig FR = new ModuleConfig(Constants.CAN.FR_DRIVE, Constants.CAN.FR_STEER, 0,
                        false, 13.4); // 9
        public final static ModuleConfig BL = new ModuleConfig(Constants.CAN.BL_DRIVE, Constants.CAN.BL_STEER, 3,
                        false, 150.2); // 147.4
        public final static ModuleConfig BR = new ModuleConfig(Constants.CAN.BR_DRIVE, Constants.CAN.BR_STEER, 2,
                        false, 323.2); // 322
        // 2.6
                }

