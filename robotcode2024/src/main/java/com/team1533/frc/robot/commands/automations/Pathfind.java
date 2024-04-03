// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD license file in the root directory of this project. 

package com.team1533.frc.robot.commands.automations;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Pathfind extends Command {

  public Pose2d targetPose2d;
  public PathPlannerPath goalPath;
  public double endVelocity;
  public double rotDelayDistance;
  private SwerveDrive m_SwerveDrive;
  private Command pathfindingCommand;

  /**
   * Creates a new Approach.
   * Robot will drive to specified point (parameter)
   **/
  public Pathfind(Pose2d targetPose2d, double endVelocity, double rotDelayDistance, SwerveDrive m_SwerveDrive) {

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_SwerveDrive);

    this.m_SwerveDrive = m_SwerveDrive;
    this.targetPose2d = targetPose2d;
  }

  public Pathfind(PathPlannerPath goalPath, double rotDelayDistance, SwerveDrive m_SwerveDrive) {
    addRequirements(m_SwerveDrive);

    this.m_SwerveDrive = m_SwerveDrive;
    this.goalPath = goalPath;

  }
  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    if (targetPose2d != null) {
      pathfindingCommand = AutoBuilder.pathfindToPose(targetPose2d, Constants.AutoAlign.constraints,
          endVelocity, // Goal end velocity in meters/sec
          rotDelayDistance // Rotation delay distance in meters. This is how far the robot should travel
                           // before attempting to rotate.
      );

    }

    else if (goalPath != null) {
      pathfindingCommand = AutoBuilder.pathfindThenFollowPath(goalPath,
          Constants.AutoAlign.constraints,
          rotDelayDistance);
    }

    pathfindingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathfindingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindingCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}