// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.commands.automations;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveTo extends Command {

  public Pose2d targetPose2d;
  public PathPlannerPath goalPath;
  public double endVelocity;
  public double rotDelayDistance;
  private SwerveDrive m_SwerveDrive;
  private Robot m_Robot;
  private Command pathfindingCommand;

  /**
   * Creates a new Approach.
   * Robot will drive to specified point (parameter)
   **/
  public DriveTo(Pose2d targetPose2d, double endVelocity, double rotDelayDistance, SwerveDrive m_SwerveDrive,
      Robot m_Robot) {

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_SwerveDrive);

    this.m_SwerveDrive = m_SwerveDrive;
    this.m_Robot = m_Robot;
    this.targetPose2d = targetPose2d;
  }

  public DriveTo(PathPlannerPath goalPath, double rotDelayDistance, SwerveDrive m_SwerveDrive, Robot m_Robot) {
    addRequirements(m_SwerveDrive);

    this.m_SwerveDrive = m_SwerveDrive;
    this.m_Robot = m_Robot;
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