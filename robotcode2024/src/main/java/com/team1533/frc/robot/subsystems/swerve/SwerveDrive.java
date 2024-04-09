package com.team1533.frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.DifferentialControlModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.RobotContainer;
import com.team1533.frc.robot.subsystems.leds.Leds.LedMode;
import com.team1533.frc.robot.subsystems.vision.VisionConstants;
import com.team1533.lib.control.AutoAlignController;
import com.team1533.lib.control.HeadingController;
import com.team1533.lib.control.AutoAlignController.AutoAlignControllerState;
import com.team1533.lib.control.HeadingController.HeadingControllerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrive extends SubsystemBase {
  private final RobotContainer m_RobotContainer;
  public double rotationPreset = 0;
  public boolean presetEnabled = false;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  private final HeadingController headingController;
  private final AutoAlignController autoAlignController;

  public static enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    TELEOP,
    /** Driving to a location on the field automatically. */
    AUTO_ALIGN,
    /** Driving with heading locked */
    AUTONOMOUS_HEADING_LOCKED,
    /** Ensures the robot does not move too fast while shooting. */
    SHOOTING,
    /** Autonoumous */
    TRAJECTORY,
    /** Running wheel radius characterization routine (spinning in circle) */
    WHEEL_RADIUS_CHARACTERIZATION
  }

  @AutoLogOutput
  @Getter
  @Setter
  private DriveMode currentDriveMode = DriveMode.TELEOP;
  @AutoLogOutput
  private SwerveModuleState[] desiredSwerveModuleStates;
  @AutoLogOutput
  private SwerveModuleState[] currentSwerveModuleStates;
  @AutoLogOutput
  public ChassisSpeeds currentMovement;
  @AutoLogOutput
  public ChassisSpeeds desiredMovement;
  @AutoLogOutput
  public double characterizationInput = 0.0;

  public SwerveDrivePoseEstimator m_odometry;

  public double controllerX = 0;
  public double controllerY = 0;
  public double controllerOmega = 0;
  public boolean robotRelative = false;

  /**
   * Creates a new DriveSubsystem.
   */
  public SwerveDrive(RobotContainer m_RobotContainer, ModuleIO FL, ModuleIO FR, ModuleIO BL, ModuleIO BR,
      GyroIO gyroIO) {
    this.m_RobotContainer = m_RobotContainer;

    this.gyroIO = gyroIO;

    headingController = new HeadingController(this);
    autoAlignController = new AutoAlignController(this);

    modules[0] = new Module(FL, 0);
    modules[1] = new Module(FR, 1);
    modules[2] = new Module(BL, 2);
    modules[3] = new Module(BR, 3);

    resetEncoders();

    // Odometry class for tracking robot pose with vision
    m_odometry = new SwerveDrivePoseEstimator(
        SwerveConstants.kDriveKinematics,
        getGyroInRotations(),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        new Pose2d(0, 0, new Rotation2d(0)),
        VisionConstants.STATE_STD_DEVS,
        VisionConstants.VISION_MEASUREMENT_STD_DEVS);

    resetEncoders();

    // Odometry class for tracking robot pose with vision
    m_odometry = new SwerveDrivePoseEstimator(
        SwerveConstants.kDriveKinematics,
        getGyroInRotations(),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        new Pose2d(0, 0, new Rotation2d(0)),
        VisionConstants.STATE_STD_DEVS,
        VisionConstants.VISION_MEASUREMENT_STD_DEVS);

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::acceptAutonomousInput, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        Constants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

  }

  // The gyro sensor
  public final double navXPitch() {
    return gyroInputs.pitchPosition.getDegrees();

  }

  public final double navXRoll() {
    return gyroInputs.rollPosition.getDegrees();
  }

  boolean gyroReset;

  public Boolean isAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getGyroInRotations() {
    // Negating the angle because WPILib gyros are CW positive.
    return gyroInputs.yawPosition.times(SwerveConstants.kGyroReversed ? 1.0 : -1.0);
  }

  public double getGyroTotalDegrees() {
    return gyroInputs.totalDistanceYawDegrees;
  }

  public boolean getGyroReset() {
    return gyroReset;
  }

  public void setGyroReset(boolean gyroReset) {
    this.gyroReset = gyroReset;
  }

  public Boolean isMovingXY() {
    return (currentMovement.vxMetersPerSecond < 2) && (currentMovement.vyMetersPerSecond < 2);
  }

  public void acceptTeleopInput(double x, double y, double omega, boolean robotRelative) {
    controllerX = x;
    controllerY = y;
    controllerOmega = omega;
    this.robotRelative = robotRelative;
    if (DriverStation.isTeleopEnabled()) {
      if (currentDriveMode != DriveMode.AUTO_ALIGN && currentDriveMode != DriveMode.SHOOTING) {
        currentDriveMode = DriveMode.TELEOP;
      }
    }
  }

  public void acceptAutonomousInput(ChassisSpeeds chassisSpeeds) {
    desiredMovement = chassisSpeeds;
    currentDriveMode = DriveMode.TRAJECTORY;
  }

  @Override
  public void periodic() {

    Arrays.stream(modules).forEach(Module::updateInputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    updateOdometry();
    updateChassisSpeeds();
    updateSwerveModuleStates();

    gyroIO.addOffset(getChassisSpeeds());

    switch (currentDriveMode) {
      case TELEOP:
      case SHOOTING:

        if (isAllianceRed()) {
          desiredMovement = ChassisSpeeds.fromFieldRelativeSpeeds(controllerX, controllerY, controllerOmega,
              getPose().getRotation().plus(Rotation2d.fromDegrees(180)));
        } else {
          desiredMovement = ChassisSpeeds.fromFieldRelativeSpeeds(controllerX, controllerY, controllerOmega,
              getPose().getRotation());
        }

        if (headingController.getM_HeadingControllerState() != HeadingController.HeadingControllerState.OFF) {
          desiredMovement.omegaRadiansPerSecond = headingController.update();
        }

        if (currentDriveMode == DriveMode.SHOOTING) {
          desiredMovement.vxMetersPerSecond = MathUtil.clamp(desiredMovement.vxMetersPerSecond, -1, 1);
          desiredMovement.vxMetersPerSecond = MathUtil.clamp(desiredMovement.vyMetersPerSecond, -1, 1);
        }
        break;

      case AUTO_ALIGN:
        desiredMovement = autoAlignController.update();
        break;

      case AUTONOMOUS_HEADING_LOCKED:
        desiredMovement = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, headingController.update(),
            getPose().getRotation());

        break;
      case TRAJECTORY:
        break;
      case WHEEL_RADIUS_CHARACTERIZATION:
        desiredMovement = new ChassisSpeeds(0, 0, characterizationInput);
        break;
      default:
        break;

    }
    setChassisSpeeds(desiredMovement);

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @AutoLogOutput
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    desiredSwerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot,
            isAllianceRed() ? getPose().getRotation().plus(Rotation2d.fromDegrees(180)) : getPose().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredSwerveModuleStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    modules[0].setDesiredState(desiredSwerveModuleStates[0]);
    modules[1].setDesiredState(desiredSwerveModuleStates[1]);
    modules[2].setDesiredState(desiredSwerveModuleStates[2]);
    modules[3].setDesiredState(desiredSwerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);

    desiredSwerveModuleStates = desiredStates;
  }

  public Rotation2d getModuleRotations() {
    double totalDistanceTraveledInRadians = (Math.abs(modules[0].getPosition().distanceMeters)
        + Math.abs(modules[1].getPosition().distanceMeters)
        + Math.abs(modules[2].getPosition().distanceMeters)
        + Math.abs(modules[3].getPosition().distanceMeters))
        / (4.0 * SwerveConstants.kDriveBaseRadius);
    return Rotation2d.fromRadians(totalDistanceTraveledInRadians);
  }

  /** Runs in a circle at omega. */
  public void runWheelRadiusCharacterization(double omegaSpeed) {
    currentDriveMode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
    characterizationInput = omegaSpeed;
  }

  @AutoLogOutput
  public double getWheelOffsetExperimental() {
    double offset = getModuleRotations().getDegrees() / Math.abs(getGyroTotalDegrees());

    return offset;
  }

  public void setXWheels() {
    desiredSwerveModuleStates = new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))

    };

    modules[0].setDesiredState(desiredSwerveModuleStates[0], true);
    modules[1].setDesiredState(desiredSwerveModuleStates[1], true);
    modules[2].setDesiredState(desiredSwerveModuleStates[2], true);
    modules[3].setDesiredState(desiredSwerveModuleStates[3], true);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    modules[0].resetEncoders();
    modules[1].resetEncoders();
    modules[2].resetEncoders();
    modules[3].resetEncoders();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getGyroInRotations(),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        pose);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {

    gyroIO.reset();

    if (isAllianceRed()) {
      resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(180)));
      m_RobotContainer.m_vision.m_field.setRobotPose(m_odometry.getEstimatedPosition());
    } else {
      resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(0)));
      m_RobotContainer.m_vision.m_field.setRobotPose(m_odometry.getEstimatedPosition());
    }

    gyroReset = true;
  }

  public void updateOdometry() {
    m_odometry.update(
        getGyroInRotations(),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
  }

  public void updateChassisSpeeds() {
    currentMovement = SwerveConstants.kDriveKinematics.toChassisSpeeds(
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState());
  }

  public void updateSwerveModuleStates() {
    currentSwerveModuleStates = new SwerveModuleState[] {
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState()
    };
  }

  @AutoLogOutput
  public ChassisSpeeds getChassisSpeeds() {
    return currentMovement;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getGyroInDeg() {
    return gyroInputs.yawPosition.getDegrees();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getYawRadPerSec() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  public void zeroWheels() {
    modules[0].resetWheel();
    modules[1].resetWheel();
    modules[2].resetWheel();
    modules[3].resetWheel();
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods

  public void setHeadingController(HeadingControllerState state, Supplier<Rotation2d> desiredHeading) {
    headingController.setM_HeadingControllerState(state);
    headingController.setGoal(desiredHeading);
    currentDriveMode = DriveMode.SHOOTING;
  }

  public void setHeadingController(Supplier<Rotation2d> desiredHeading) {
    headingController.setM_HeadingControllerState(HeadingControllerState.SNAP);
    headingController.setGoal(desiredHeading);
    currentDriveMode = DriveMode.SHOOTING;
  }

  public void setHeadingController(HeadingControllerState state, Rotation2d desiredHeading) {
    headingController.setM_HeadingControllerState(state);
    headingController.setGoal(desiredHeading);
  }

  public void setHeadingController(Rotation2d desiredHeading) {
    headingController.setM_HeadingControllerState(HeadingControllerState.SNAP);
    headingController.setGoal(desiredHeading);
  }

  public void setHeadingControllerInAuto(HeadingControllerState state, Supplier<Rotation2d> desiredHeading) {
    headingController.setM_HeadingControllerState(state);
    headingController.setGoal(desiredHeading);
    currentDriveMode = DriveMode.AUTONOMOUS_HEADING_LOCKED;
  }

  public void setHeadingControllerInAuto(Supplier<Rotation2d> desiredHeading) {
    headingController.setM_HeadingControllerState(HeadingControllerState.SNAP);
    headingController.setGoal(desiredHeading);
    currentDriveMode = DriveMode.AUTONOMOUS_HEADING_LOCKED;
  }

  public void setHeadingControllerInAuto(HeadingControllerState state, Rotation2d desiredHeading) {
    headingController.setM_HeadingControllerState(state);
    headingController.setGoal(desiredHeading);
    currentDriveMode = DriveMode.AUTONOMOUS_HEADING_LOCKED;
  }

  public void setHeadingControllerInAuto(Rotation2d desiredHeading) {
    headingController.setM_HeadingControllerState(HeadingControllerState.SNAP);
    headingController.setGoal(desiredHeading);
    currentDriveMode = DriveMode.AUTONOMOUS_HEADING_LOCKED;
  }

  public void disableHeadingController() {
    headingController.setM_HeadingControllerState(HeadingControllerState.OFF);
    if (currentDriveMode == DriveMode.AUTONOMOUS_HEADING_LOCKED && currentDriveMode != DriveMode.TRAJECTORY) {
      currentDriveMode = DriveMode.TELEOP;
    }
  }

  public HeadingControllerState getHeadingControllerState() {
    return headingController.getM_HeadingControllerState();
  }

  public Rotation2d getHeadingControllerGoal() {
    return headingController.getGoal();
  }

  public void setAutoAlignController(AutoAlignControllerState state, Supplier<Pose2d> desiredPose) {
    m_RobotContainer.m_Leds.setMode(LedMode.AUTO_ALIGN);
    autoAlignController.setM_AutoAlignControllerState(state);
    autoAlignController.setGoal(desiredPose);
    currentDriveMode = DriveMode.AUTO_ALIGN;

  }

  public void setAutoAlignController(Supplier<Pose2d> desiredPose) {
    m_RobotContainer.m_Leds.setMode(LedMode.AUTO_ALIGN);
    autoAlignController.setM_AutoAlignControllerState(AutoAlignControllerState.AUTO_ALIGN_FAST);
    autoAlignController.setGoal(desiredPose);
    currentDriveMode = DriveMode.AUTO_ALIGN;

  }

  public void setAutoAlignController(AutoAlignControllerState state, Pose2d desiredPose) {
    m_RobotContainer.m_Leds.setMode(LedMode.AUTO_ALIGN);
    autoAlignController.setM_AutoAlignControllerState(state);
    autoAlignController.setGoal(desiredPose);
    currentDriveMode = DriveMode.AUTO_ALIGN;

  }

  public void setAutoAlignController(Pose2d desiredPose) {
    m_RobotContainer.m_Leds.setMode(LedMode.AUTO_ALIGN);
    autoAlignController.setM_AutoAlignControllerState(AutoAlignControllerState.AUTO_ALIGN_FAST);
    autoAlignController.setGoal(desiredPose);
    currentDriveMode = DriveMode.AUTO_ALIGN;

  }

  public void setAutoAlignController(Translation2d desiredTranslation2d) {
    m_RobotContainer.m_Leds.setMode(LedMode.AUTO_ALIGN);
    autoAlignController.setM_AutoAlignControllerState(AutoAlignControllerState.AUTO_ALIGN_FAST);
    autoAlignController
        .setGoal(new Pose2d(desiredTranslation2d, desiredTranslation2d.minus(getPose().getTranslation()).getAngle()));
    currentDriveMode = DriveMode.AUTO_ALIGN;
  }

  public void disableAutoAlignController() {
    m_RobotContainer.m_Leds.setMode(LedMode.DEFAULT);
    autoAlignController.setM_AutoAlignControllerState(AutoAlignControllerState.OFF);
  }

  public AutoAlignControllerState getAutoAlignControllerState() {
    return autoAlignController.getM_AutoAlignControllerState();
  }

  public Pose2d getAutoAlignControllerGoal() {
    return autoAlignController.getGoal();
  }

  public void updateSmartDashBoard() {

    SmartDashboard.putNumber("x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("y", getPose().getTranslation().getY());
    SmartDashboard.putNumber("r", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("GYRO ANGLE", gyroInputs.yawPosition.getDegrees());
    SmartDashboard.putNumber("wheel offset", getWheelOffsetExperimental());

    SmartDashboard.putNumber("BRsteering", modules[0].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("FRsteering", modules[1].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("FLsteering", modules[2].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("BLsteering", modules[3].getPosition().angle.getDegrees());

    ;
  }

}