package com.team1533.frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team1533.frc.robot.util.Alert;
import com.team1533.frc.robot.util.Alert.AlertType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class FieldPositions {

    public static final Pose2d AMP = new Pose2d(new Translation2d(1.82, 7.68), new Rotation2d().fromDegrees(90));
  }

  public static final class LoggerConstants {

    private static RobotType robotType = RobotType.COMPBOT;

    public static final Boolean tuningMode = false;
    public final static double kDt = 0.02;

    public static RobotType getRobot() {
      if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
            .set(true);
        robotType = RobotType.COMPBOT;
      }
      return robotType;
    }

    public static Mode getMode() {
      return switch (robotType) {
        case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        case SIMBOT -> Mode.SIM;
      };
    }

    public enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    public enum RobotType {
      SIMBOT,
      DEVBOT,
      COMPBOT
    }

    /** Checks whether the robot the correct robot is selected when deploying. */
    public static void main(String... args) {
      if (robotType == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPTranslationController = 3;
    public static final double kDTranslationController = 0.3;

    public static final double kPThetaController = 3;

    public static final boolean enableInitialReplanning = false;
    public static final boolean enableDynamicReplanning = false;
    public static final double dynamicReplanningTotalErrorThreshold = 0.4572;
    public static final double dynamicReplanningErrorSpikeThreshold = 3;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(kPTranslationController, 0.0, kDTranslationController), // Translation PID constants
        new PIDConstants(kPThetaController, 0.0, 0.0), // Rotation PID constants
        Constants.AutoAlign.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(enableInitialReplanning,
            enableDynamicReplanning,
            dynamicReplanningTotalErrorThreshold,
            dynamicReplanningErrorSpikeThreshold) // Default path replanning config. See the API for the options here
    );
  }

  public static final class AutoAlign {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final PathConstraints constraints = new PathConstraints(
        1.0, 2.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  public static final class ELECTRICAL {
    public static final int swerveTurningCurrentLimit = 40;
    public static final int swerveDrivingCurrentLimit = 40;

    public static final int elevatorCurrentLimit = 25;
    public static final int intakeCurrentLimit = 30;

    public static final int rollerCurrentLimit = 40;
    public static final int indexerCurrentLimit = 20;

    public static final int climbCurrentLimit = 40;

    public static final int indexerDigitalInput = 4;
    public static final int intakeDigitalInput = 1;

    public static final int pivotAbsInput = 2;
    public static final int intakeAbsInput = 3;

    public static final int shooterPivotCurrentLimit = 40;
    public static final int flyWheelCurrentLimit = 30;
  }

  public static final class CAN {
    // Swerve Motor Controller CAN ID's
    public static final int FL_DRIVE = 3;
    public static final int FR_DRIVE = 11;
    public static final int BL_DRIVE = 20;
    public static final int BR_DRIVE = 13;
    public static final int FL_STEER = 4;
    public static final int FR_STEER = 12;
    public static final int BL_STEER = 1;
    public static final int BR_STEER = 14;

    public static final int ELEVATOR = 9;

    public static final int ROLLERS = 5;
    public static final int INDEXER = 10;

    public static final int IPIVOT = 23;

    // flywheel L/R from intake perspective
    public static final int FLYWHEELL = 7;
    public static final int FLYWHEELR = 6;
    public static final int PIVOTL = 15;
    public static final int PIVOTR = 16;

    public static final int CLIMBL = 17;
    public static final int CLIMBR = 8;

  }

  public static class JoystickButtons {
    public static final XboxController m_driverController = new XboxController(0);
    public static final XboxController m_operatorController = new XboxController(1);

    public static final JoystickButton opA = new JoystickButton(m_operatorController, 1);
    public static final JoystickButton opB = new JoystickButton(m_operatorController, 2);
    public static final JoystickButton opX = new JoystickButton(m_operatorController, 3);
    public static final JoystickButton opY = new JoystickButton(m_operatorController, 4);
    public static final JoystickButton oplBump = new JoystickButton(m_operatorController, 5);
    public static final JoystickButton oprBump = new JoystickButton(m_operatorController, 6);
    public static final JoystickButton oplWing = new JoystickButton(m_operatorController, 7);
    public static final JoystickButton oprWing = new JoystickButton(m_operatorController, 8);
    public static final JoystickButton oplJoy = new JoystickButton(m_operatorController, 9);
    public static final JoystickButton oprJoy = new JoystickButton(m_operatorController, 10);
    public static final POVButton opDpadD = new POVButton(m_operatorController, 180);
    public static final POVButton opDpadU = new POVButton(m_operatorController, 0);
    public static final POVButton opDpadL = new POVButton(m_operatorController, 270);
    public static final POVButton opDpadR = new POVButton(m_operatorController, 90);

    public static final JoystickButton dA = new JoystickButton(m_driverController, 1);
    public static final JoystickButton dB = new JoystickButton(m_driverController, 2);
    public static final JoystickButton dX = new JoystickButton(m_driverController, 3);
    public static final JoystickButton dY = new JoystickButton(m_driverController, 4);
    public static final JoystickButton dlBump = new JoystickButton(m_driverController, 5);
    public static final JoystickButton drBump = new JoystickButton(m_driverController, 6);
    public static final JoystickButton dlWing = new JoystickButton(m_driverController, 7);
    public static final JoystickButton drWing = new JoystickButton(m_driverController, 8);
    public static final JoystickButton dlJoy = new JoystickButton(m_driverController, 9);
    public static final JoystickButton drJoy = new JoystickButton(m_driverController, 10);
    public static final POVButton dDpadD = new POVButton(m_driverController, 180);
    public static final POVButton dDpadU = new POVButton(m_driverController, 0);
    public static final POVButton dDpadL = new POVButton(m_driverController, 270);
    public static final POVButton dDpadR = new POVButton(m_driverController, 90);
  }

}