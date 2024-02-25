// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveTo extends Command {

  public double xAutoSpeed = 0;
  public double yAutoSpeed = 0;
  public double rAutoSpeed = 0;
  public Pose2d pose2d;

  public PIDController transformX = new PIDController(Constants.AutoAlign.kPXController,
      Constants.AutoAlign.kIXController, Constants.AutoAlign.kDXController);
  public PIDController transformY = new PIDController(Constants.AutoAlign.kPYController,
      Constants.AutoAlign.kIYController, Constants.AutoAlign.kDYController);
  public PIDController rotation = new PIDController(Constants.AutoAlign.kPThetaController,
      Constants.AutoAlign.kIThetaController, Constants.AutoAlign.kDThetaController);

  private SwerveDrive m_SwerveDrive;
  private Robot m_Robot;

  /** Creates a new Approach. */
  /*
   * Robot will drive to specified point (parameter)
   */

  public DriveTo(Pose2d pose2d, SwerveDrive m_SwerveDrive, Robot m_Robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveDrive);
    this.m_SwerveDrive = m_SwerveDrive;
    this.m_Robot = m_Robot;
    this.pose2d = pose2d;
    rotation.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xAutoSpeed = transformX.calculate(m_SwerveDrive.getPose().getX(), pose2d.getX());
    yAutoSpeed = transformY.calculate(m_SwerveDrive.getPose().getY(), pose2d.getY());
    rAutoSpeed = rotation.calculate(m_SwerveDrive.getAngle().getRadians(), pose2d.getRotation().getRadians());
    // Max Speeds
    xAutoSpeed = MathUtil.clamp(xAutoSpeed, -SwerveConstants.autoAlignMaxSpeedMetersPerSecond,
        SwerveConstants.autoAlignMaxSpeedMetersPerSecond);
    yAutoSpeed = MathUtil.clamp(yAutoSpeed, -SwerveConstants.autoAlignMaxSpeedMetersPerSecond,
        SwerveConstants.autoAlignMaxSpeedMetersPerSecond);
    rAutoSpeed = MathUtil.clamp(rAutoSpeed, -SwerveConstants.autoAlignRotationalMaxSpeedMetersPerSecond,
        SwerveConstants.autoAlignRotationalMaxSpeedMetersPerSecond);

    m_SwerveDrive.drive(xAutoSpeed, yAutoSpeed, rAutoSpeed, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Math.abs(m_SwerveDrive.getPose().getX() - pose2d.getX()) <= 0.05) &&
        (Math.abs(m_SwerveDrive.getPose().getY() - pose2d.getY()) <= 0.05) &&
        (Math.abs(m_SwerveDrive.getPose().getRotation().getRadians() -
            pose2d.getRotation().getRadians()) <= 0.02)) {
      return true;
    }
    return false;
  }
}