// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD license file in the root directory of this project. 


package frc.robot.commands.automations; 
import javax.lang.model.util.ElementScanner14;
import javax.swing.text.GlyphView.GlyphPainter;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder; 
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants; 
import frc.robot.Constants.SwerveConstants; 
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrive; 

public class Shoot { 
  
  /** Creates a new Approach.
   *  Robot will drive to specified point (parameter) 
  **/ 

    RobotContainer m_RobotContainer;

    // these are the points used for stage
    // https://imgur.com/a/OeuetIS
    Translation2d z = new Translation2d(230.763, 107.484);
    Translation2d x = new Translation2d(230.763, 215.793);
    Translation2d c = new Translation2d(218.371193, 100.579008);
    Translation2d v = new Translation2d(218.371193, 223.197810);
    Translation2d b = new Translation2d(124.572, 168.793110);
    Translation2d n = new Translation2d(124.572, 154.483709);


    Translation2d robotPose2dInches;
        
    double lowerY = 323.576819;
    double upperY = 0;
        
    // slopes
    double m;
    double y;

    Boolean canShoot;

    // variables for shooting

    Rotation2d shootingRotation;
    double shootingAngle = 0;
    Translation3d speakerPose3d = new Translation3d(0.3603998634, 5.6282082, 2.1326856);
    

    public Shoot(RobotContainer m_RobotContainer) {
    // Use addRequirements() here to declare subsystem dependencies. 

    this.m_RobotContainer = m_RobotContainer;
    } 

    public void prepare()   {
        if (isAllianceRed())    {
        shootingRotation = new Translation2d(flipTranslation3d(speakerPose3d).getX(), flipTranslation3d(speakerPose3d).getY()) .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle();
        }

        shootingRotation = new Translation2d(speakerPose3d.getX(), speakerPose3d.getY()) .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle();

        m_RobotContainer.m_robotDrive.setPresetEnabled(true, shootingRotation.getDegrees());
        m_RobotContainer.m_flywheel.setFWSpeed(-5676);
    }

    public void execute()   {

        shootingAngle = Units.radiansToDegrees(Math.atan2(speakerPose3d.getZ(), Math.hypot(m_RobotContainer.m_robotDrive.getPose().getX(), m_RobotContainer.m_robotDrive.getPose().getY()))) - 90 + 32.5;
        m_RobotContainer.m_shooter.setShooterAngle(shootingAngle);
    } 

    public void shoot()  {
        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose()) && pivotCheck() && flyWheelCheck())    {

            m_RobotContainer.m_conveyor.runConvIn();
        }
    }
        
    public Boolean swerveCheck(Pose2d robotPose2d) { 

        if (isAllianceRed())    {
            robotPose2dInches = new Translation2d(Units.metersToInches(flipPose(robotPose2d).getX()), Units.metersToInches(flipPose(robotPose2d).getY()));
        } else  {
        robotPose2dInches = new Translation2d(Units.metersToInches(robotPose2d.getX()), Units.metersToInches(robotPose2d.getY()));
        }
        if (robotPose2dInches.getX() > 419.584)  {
            canShoot = false;
        }   else if (robotPose2dInches.getX() >= 230.763)  {
            lowerY = 107.484;
            upperY = 215.793;
        }   else if(robotPose2dInches.getX() <= 124.572)  {
            lowerY = 154.483709;
            upperY = 168.793110;
        }   else if(124.572 <= robotPose2dInches.getX() && robotPose2dInches.getX() <= 218.371) {
            m = (v.getY() - b.getY()) / (v.getX() - b.getX()); 
            upperY = m * (robotPose2dInches.getX() - v.getX()) + v.getY();

            y = (c.getY() - n.getY()) / (c.getX() - n.getX()); 
            lowerY = y * (robotPose2dInches.getX() - c.getX()) + c.getY();
        }   else {
            m = (x.getY() - v.getY()) / (x.getX() - v.getX()); 
            upperY = m * (robotPose2dInches.getX() - x.getX()) + x.getY();

            y = (c.getY() - z.getY()) / (c.getX() - z.getX()); 
            lowerY = y * (robotPose2dInches.getX() - c.getX()) + c.getY();
        }


        canShoot = (lowerY >= robotPose2dInches.getY()) && (robotPose2dInches.getY() >= upperY);

        return canShoot;
    } 
    
    public Boolean pivotCheck() { 
        return Math.abs(m_RobotContainer.m_shooter.getAngle() - shootingAngle) < 0.1;
    } 
    
    public Boolean flyWheelCheck() { 
        return Math.abs(m_RobotContainer.m_flywheel.getLeftSpeed() - (-5676)) < 100;
    } 

    public Pose2d flipPose(Pose2d pose)    {
        return new Pose2d(16.54 - pose.getX(), pose.getY(), pose.getRotation().rotateBy(new Rotation2d().fromRadians(Math.PI/2)));
    }

    public Translation3d flipTranslation3d(Translation3d translation)   {
        return new Translation3d(16.54 - translation.getX(), translation.getY(), translation.getZ());

    }

    public boolean isAllianceRed()  {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
    }
    
  }