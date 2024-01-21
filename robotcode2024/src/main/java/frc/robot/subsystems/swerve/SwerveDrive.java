package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrive extends SubsystemBase {
  private final Robot m_Robot;
  public double rotationPreset = 0;
  public boolean presetEnabled = false;

  Pose2d visionPoseLeft = new Pose2d();
  Pose2d visionPoseRight = new Pose2d();

  public Field2d m_field = new Field2d();


  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(CAN.FL_DRIVE,
      CAN.FL_STEER,
      ModuleConstants.FL_ENCODER,
      SwerveConstants.frontLeftSteerEncoderReversed,
      ModuleConstants.FL_ENC_OFFSET);

  private final SwerveModule m_rearLeft = new SwerveModule(CAN.BL_DRIVE,
      CAN.BL_STEER,
      ModuleConstants.BL_ENCODER,
      SwerveConstants.backLeftSteerEncoderReversed,
      ModuleConstants.BL_ENC_OFFSET);

  private final SwerveModule m_frontRight = new SwerveModule(CAN.FR_DRIVE,
      CAN.FR_STEER,
      ModuleConstants.FR_ENCODER,
      SwerveConstants.frontRightSteerEncoderReversed,
      ModuleConstants.FR_ENC_OFFSET);

  private final SwerveModule m_rearRight = new SwerveModule(CAN.BR_DRIVE,
      CAN.BR_STEER,
      ModuleConstants.BR_ENCODER,
      SwerveConstants.backRightSteerEncoderReversed,
      ModuleConstants.BR_ENC_OFFSET);

  private SwerveModuleState[] swerveModuleStates;
  public ChassisSpeeds currentMovement;

  // The gyro sensor
  public final double navXPitch() {
    return navX.getPitch();

  }

  public final double navXRoll() {
    return navX.getRoll();
  }

  private static final AHRS navX = new AHRS(SPI.Port.kMXP);
  boolean gyroReset;

  // Odometry class for tracking robot pose with vision
  public SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      SwerveConstants.kDriveKinematics,
      getAngle(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d(0, 0, new Rotation2d(0)),
      Constants.VisionConstants.STATE_STD_DEVS,
      Constants.VisionConstants.VISION_MEASUREMENT_STD_DEVS);

  /**
   * Creates a new DriveSubsystem.
   */
  public SwerveDrive(Robot m_robot) {
    resetEncoders();
    m_Robot = m_robot;
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees((navX.getAngle()) * (SwerveConstants.kGyroReversed ? 1.0 : -1.0));
  }

 
  public boolean getGyroReset() {
    return gyroReset;
  }

  public void setGyroReset(boolean gyroReset) {
    this.gyroReset = gyroReset;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block=
    m_odometry.update(
        getAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    updateOdometry();

    // System.out.print("xSpeed: " + xAutoSpeed + ";\n ySpeed: " + yAutoSpeed + ";\n
    // rSpeed: " + rAutoSpeed);
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
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
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
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setXWheels() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navX.reset();
    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(0)));
    m_field.setRobotPose(m_odometry.getEstimatedPosition());

    gyroReset = true;
  }
  // TODO: why am i still here just to suffer
  // Calculates closest Apriltag for use in autoAlignCube
  public int optimalID() {
    Pose2d robotPose = getPose();
    return 0;
  }
  // TODO: fix this
  public void updateOdometry() {

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public static double getHeading() {
    double heading = Math.IEEEremainder(navX.getAngle(), 360) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
    return heading;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void zeroWheels() {
    m_frontLeft.resetWheel();
    m_rearLeft.resetWheel();
    m_frontRight.resetWheel();
    m_rearRight.resetWheel();
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
    SmartDashboard.putNumber("FLSteering", m_frontLeft.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("FRSteering", m_frontRight.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("BLSteering", m_rearLeft.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("BRSteering", m_rearRight.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("FLSteeringDeg", m_frontLeft.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("FRSteeringDeg", m_frontRight.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("BLSteeringDeg", m_rearLeft.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("BRSteeringDeg", m_rearRight.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("FLSteerNEO", m_frontLeft.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("FRSteerNEO", m_frontRight.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("BLSteerNEO", m_rearLeft.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("BRSteerNEO", m_rearRight.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("FLneo", m_frontLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("FRneo", m_frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("BLneo", m_rearLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("BRneo", m_rearRight.getState().angle.getRadians());
    SmartDashboard.putNumber("x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("y", getPose().getTranslation().getY());
    SmartDashboard.putNumber("r", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("GYRO ANGLE", navX.getAngle());
    SmartDashboard.putNumber("TurnRate", getTurnRate());
    // SmartDashboard.putNumber("Has Target?", tv);
    // SmartDashboard.putNumber("xSpeed", xAutoSpeed);
    // SmartDashboard.putNumber("ySpeed", yAutoSpeed);
    // SmartDashboard.putNumber("rSpeed", rAutoSpeed);
    SmartDashboard.putNumber("pitch", navX.getPitch());
    SmartDashboard.putNumber("roll", navX.getRoll());
    SmartDashboard.putNumber("yaw", navX.getYaw());
    SmartDashboard.putString("Alliance Color", m_Robot.allianceColor.toString());
    // if (tempRobotPose.length >= 7) {
    // SmartDashboard.putNumber("Vision x", tempRobotPose[0]);
    // SmartDashboard.putNumber("Vision y", tempRobotPose[1]);
    // SmartDashboard.putNumber("Vision r", tempRobotPose[5]);
    // }
    
    // SmartDashboard.putNumber("tempRobotPose length", tempRobotPose.length);

    SmartDashboard.putData("Field", m_field);
  }
}