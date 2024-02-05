// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoAlign extends Command {

  public double xAutoSpeed = 0;
  public double yAutoSpeed = 0;
  public double rAutoSpeed = 0;
  public double offset;
  public PIDController transformX = new PIDController(5, 0.01, 0);
  public PIDController transformY = new PIDController(3, 0.01, 0);
  public PIDController rotation = new PIDController(7, 0.01, 0);
  private SwerveDrive m_SwerveDrive;
  private Robot m_Robot;
  private Pose2d tagPose;
  public Pose2d targetPose;

  /** Creates a new Approach. */
  /*
   * Robot will drive to specified point (parameter)
   */

  public AutoAlign(double offset, SwerveDrive m_SwerveDrive, Robot m_Robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveDrive);
    this.m_SwerveDrive = m_SwerveDrive;
    this.m_Robot = m_Robot;
    this.offset = offset;
    rotation.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}