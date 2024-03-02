// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD license file in the root directory of this project. 


package frc.robot.commands.automations; 
import com.pathplanner.lib.auto.AutoBuilder; 
import com.pathplanner.lib.commands.PathfindingCommand; 
import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants; 
import frc.robot.Constants.SwerveConstants; 
import frc.robot.Robot; 
import frc.robot.subsystems.swerve.SwerveDrive; 

public class DriveTo extends Command { 

  public Pose2d targetPose2d; 
  public double endVelocity; 
  public double rotDelayDistance; 
  private SwerveDrive m_SwerveDrive; 
  private Robot m_Robot; 
  private Command pathfindingCommand; 
  
  /** Creates a new Approach.
   *  Robot will drive to specified point (parameter) 
  **/ 
  public DriveTo(Pose2d targetPose2d, double endVelocity, double rotDelayDistance, SwerveDrive m_SwerveDrive, Robot m_Robot) { 
    
    // Use addRequirements() here to declare subsystem dependencies. 
    
    addRequirements(m_SwerveDrive); 
    
    this.m_SwerveDrive = m_SwerveDrive; 
    this.m_Robot = m_Robot; 
    this.targetPose2d = targetPose2d; 
  } 
    // Called when the command is initially scheduled. 
    
    @Override 
    public void initialize() { 
      pathfindingCommand = AutoBuilder.pathfindToPose(targetPose2d, Constants.AutoAlign.constraints, 
      endVelocity, // Goal end velocity in meters/sec 
      rotDelayDistance // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. 
      ); 
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