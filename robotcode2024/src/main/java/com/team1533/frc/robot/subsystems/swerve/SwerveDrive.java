package com.team1533.frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.RobotContainer;
import com.team1533.frc.robot.subsystems.vision.VisionConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrive extends SubsystemBase {
  private final RobotContainer m_RobotContainer;
  public double rotationPreset = 0;
  public boolean presetEnabled = false;

  @AutoLog
  public static class OdometryTimestampInputs {
    public double[] timestamps = new double[] {};
  }

  public static final Lock odometryLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);

  private final OdometryTimestampInputsAutoLogged odometryTimestampInputs = new OdometryTimestampInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  private SwerveModuleState[] swerveModuleStates;
  public ChassisSpeeds currentMovement;

  /**
   * Creates a new DriveSubsystem.
   */
  public SwerveDrive(RobotContainer m_RobotContainer, ModuleIO FL, ModuleIO FR, ModuleIO BL, ModuleIO BR,
      GyroIO gyroIO) {
    resetEncoders();
    this.m_RobotContainer = m_RobotContainer;

    this.gyroIO = gyroIO;

    modules[0] = new Module(FL, 0);
    modules[1] = new Module(FR, 1);
    modules[2] = new Module(BL, 2);
    modules[3] = new Module(BR, 3);

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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

  // Odometry class for tracking robot pose with vision
  public SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      SwerveConstants.kDriveKinematics,
      getAngle(),
      new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      },
      new Pose2d(0, 0, new Rotation2d(0)),
      VisionConstants.STATE_STD_DEVS,
      VisionConstants.VISION_MEASUREMENT_STD_DEVS);

  public Boolean isRedAlliance() {
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
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return gyroInputs.yawPosition.times(SwerveConstants.kGyroReversed ? 1.0 : -1.0);
  }

  public boolean getGyroReset() {
    return gyroReset;
  }

  public void setGyroReset(boolean gyroReset) {
    this.gyroReset = gyroReset;
  }

  public Boolean isMovingXY() {
    return (currentMovement.vxMetersPerSecond < 1) && (currentMovement.vyMetersPerSecond < 1);
  }

  @Override
  public void periodic() {
    // Update & process inputs
    odometryLock.lock();
    // Read timestamps from odometry thread and fake sim timestamps
    odometryTimestampInputs.timestamps = timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    if (odometryTimestampInputs.timestamps.length == 0) {
      odometryTimestampInputs.timestamps = new double[] { Timer.getFPGATimestamp() };
    }
    timestampQueue.clear();
    Logger.processInputs("Drive/OdometryTimestamps", odometryTimestampInputs);
    // Read inputs from gyro
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    // Read inputs from modules
    Arrays.stream(modules).forEach(Module::updateInputs);
    odometryLock.unlock();

    updateOdometry();
    updateChassisSpeeds();

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
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

    swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot,
            isRedAlliance() ? getPose().getRotation().plus(Rotation2d.fromDegrees(180)) : getPose().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    modules[0].setDesiredState(swerveModuleStates[0]);
    modules[1].setDesiredState(swerveModuleStates[1]);
    modules[2].setDesiredState(swerveModuleStates[2]);
    modules[3].setDesiredState(swerveModuleStates[3]);
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
  }

  public void setXWheels() {
    modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
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
        getAngle(),
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

    if (isRedAlliance()) {
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
        getAngle(),
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

  @AutoLogOutput
  public ChassisSpeeds getChassisSpeeds() {
    return currentMovement;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDeg() {
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
  public double getTurnRate() {
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

  public void setPresetEnabled(boolean enabled, double desiredHeading) {
    presetEnabled = enabled;
    rotationPreset = desiredHeading;
  }

  public void setPresetEnabled(boolean enabled) {
    presetEnabled = enabled;
  }

  public boolean getPresetEnabled() {
    return presetEnabled;
  }

  public double getRotationPreset() {
    return rotationPreset;
  }

  public void updateSmartDashBoard() {

    SmartDashboard.putNumber("x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("y", getPose().getTranslation().getY());
    SmartDashboard.putNumber("r", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("GYRO ANGLE", gyroInputs.yawPosition.getDegrees());

    SmartDashboard.putNumber("BRsteering", modules[0].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("FRsteering", modules[1].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("FLsteering", modules[2].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("BLsteering", modules[3].getPosition().angle.getDegrees());

    ;
  }

}