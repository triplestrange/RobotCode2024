// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1533.frc.robot.commands;

import com.team1533.frc.robot.Constants.JoystickButtons;
import com.team1533.frc.robot.subsystems.swerve.SwerveConstants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDrive extends Command {
  private SwerveDrive m_swerve;
  private double speed;
  private double rotationSpeed;
  private Timer timer = new Timer();
  private double deadzone;

  /**
   * Creates a new Drive.
   * 
   * normal = 2.5
   * slow = 0.75
   */
  public DefaultDrive(SwerveDrive m_swerve, double speed, double rotationSpeed) {
    addRequirements(m_swerve);
    this.m_swerve = m_swerve;
    this.speed = speed;
    this.rotationSpeed = rotationSpeed;
    speed = MathUtil.clamp(speed, -SwerveConstants.kMaxSpeedMetersPerSecond,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    deadzone = 0.2;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedR;

    if (Math.abs(JoystickButtons.m_driverController.getRightX()) > deadzone) {
      m_swerve.disableHeadingController();
    }

    double norm = Math.hypot(JoystickButtons.m_driverController.getLeftX(),
        JoystickButtons.m_driverController.getLeftY());

    double speedY = JoystickButtons.m_driverController.getLeftY() * speed;
    double speedX = JoystickButtons.m_driverController.getLeftX() * speed;

    if (Math.abs(JoystickButtons.m_driverController.getRightX()) <= deadzone) {
      speedR = 0;
    }
    // rotation was reversed
    else {
      speedR = Math.pow(JoystickButtons.m_driverController.getRightX(), 3) * -4 * rotationSpeed;
    }

    if (Math.abs(JoystickButtons.m_driverController.getLeftY()) <= deadzone) {
      speedY = 0;
    }
    if (Math.abs(JoystickButtons.m_driverController.getLeftX()) <= deadzone) {
      speedX = 0;
    }

    m_swerve.acceptTeleopInput(
        -speedY * Math.pow(norm, 2),
        -speedX * Math.pow(norm, 2),
        speedR,
        true);

    SmartDashboard.putNumber("left Y", JoystickButtons.m_driverController.getLeftY());
    SmartDashboard.putNumber("left X", JoystickButtons.m_driverController.getLeftX());
    SmartDashboard.putNumber("right X", JoystickButtons.m_driverController.getRightX());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
