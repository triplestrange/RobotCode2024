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

    /**
     * Creates a new Approach.
     * Robot will drive to specified point (parameter)
     **/

    RobotContainer m_RobotContainer;
    public DriveTo driveTo;

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

    public Boolean hasNote;

    // variables for shooting

    Rotation2d shootingRotation;
    double shootingAngle = 0;
    Translation3d speakerTranslation3d = new Translation3d(0, 5.6282082, 2 + 0.025);

    public Shoot(RobotContainer m_RobotContainer) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_RobotContainer = m_RobotContainer;

        driveTo = new DriveTo(new Pose2d(
                speakerTranslation3d.getX() + Units.inchesToMeters(16) + Units.inchesToMeters(36.241382),
                speakerTranslation3d.getY(), new Rotation2d().fromDegrees(0)), 0, 0, m_RobotContainer.m_robotDrive,
                m_RobotContainer.m_robot);

        hasNote = m_RobotContainer.m_conveyor.getConveyorSensor();
    }

    public void prepare() {
        if (isAllianceRed()) {
            shootingRotation = new Translation2d(flipTranslation3d(speakerTranslation3d).getX(),
                    flipTranslation3d(speakerTranslation3d).getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle();
        }
        else {
        shootingRotation = new Translation2d(speakerTranslation3d.getX(), speakerTranslation3d.getY())
                .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                .plus(new Rotation2d().fromDegrees(180));
        }
        m_RobotContainer.m_robotDrive.setPresetEnabled(true, shootingRotation.getDegrees());
        m_RobotContainer.m_flywheel.setFWSpeed(-5676);

        if (isAllianceRed()) {
            shootingAngle = Units.radiansToDegrees(Math.atan2(flipTranslation3d(speakerTranslation3d).getZ(), Math
                    .hypot(m_RobotContainer.m_robotDrive.getPose().getX()
                            - flipTranslation3d(speakerTranslation3d).getX(),
                            m_RobotContainer.m_robotDrive.getPose().getY()
                                    - flipTranslation3d(speakerTranslation3d).getY())))
                    - 90 + 32.5;
        } else {
            shootingAngle = Units.radiansToDegrees(Math.atan2(speakerTranslation3d.getZ(), Math
                    .hypot(m_RobotContainer.m_robotDrive.getPose().getX() - speakerTranslation3d.getX(),
                            m_RobotContainer.m_robotDrive.getPose().getY() - speakerTranslation3d.getY())))
                    - 90 + 32.5;
        }

    }

    public void execute() {

        m_RobotContainer.m_shooter.setShooterAngle(shootingAngle);
    }

    public void shoot() {
        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose()) && pivotCheck() && flyWheelCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose())) {

            m_RobotContainer.m_conveyor.runConvIn();
        }
    }

    public void autoShoot() {
        prepare();

        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose())) {
            driveTo.cancel();
            execute();
            shoot();
        } else {
            driveTo.schedule();
        }

    }

    public Boolean swerveCheck(Pose2d robotPose2d) {

        if (isAllianceRed()) {
            robotPose2dInches = new Translation2d(Units.metersToInches(flipPose(robotPose2d).getX()),
                    Units.metersToInches(flipPose(robotPose2d).getY()));
        } else {
            robotPose2dInches = new Translation2d(Units.metersToInches(robotPose2d.getX()),
                    Units.metersToInches(robotPose2d.getY()));
        }
        if (robotPose2dInches.getX() > 419.584) {
            canShoot = false;
        } else if (robotPose2dInches.getX() >= 230.763) {
            lowerY = 107.484;
            upperY = 215.793;
        } else if (robotPose2dInches.getX() <= 124.572) {
            lowerY = 154.483709;
            upperY = 168.793110;
        } else if (124.572 <= robotPose2dInches.getX() && robotPose2dInches.getX() <= 218.371) {
            m = (v.getY() - b.getY()) / (v.getX() - b.getX());
            upperY = m * (robotPose2dInches.getX() - v.getX()) + v.getY();

            y = (c.getY() - n.getY()) / (c.getX() - n.getX());
            lowerY = y * (robotPose2dInches.getX() - c.getX()) + c.getY();
        } else {
            m = (x.getY() - v.getY()) / (x.getX() - v.getX());
            upperY = m * (robotPose2dInches.getX() - x.getX()) + x.getY();

            y = (c.getY() - z.getY()) / (c.getX() - z.getX());
            lowerY = y * (robotPose2dInches.getX() - c.getX()) + c.getY();
        }

        canShoot = ((lowerY > robotPose2dInches.getY()) || (robotPose2dInches.getY() > upperY));

        return canShoot;
    }

    public Boolean pivotCheck() {
        return Math.abs(m_RobotContainer.m_shooter.getAngle() - shootingAngle) < 0.1;
    }

    public Boolean rotationCheck(Pose2d robotPose2d) {
        return robotPose2d.getRotation().minus(shootingRotation).getDegrees() < 3;
    }

    public Boolean flyWheelCheck() {
        return Math.abs(m_RobotContainer.m_flywheel.getLeftSpeed() - (-5676)) < 500;
    }

    public Pose2d flipPose(Pose2d pose) {
        return new Pose2d(16.54 - pose.getX(), pose.getY(),
                pose.getRotation().rotateBy(new Rotation2d().fromRadians(Math.PI / 2)));
    }

    public Translation3d flipTranslation3d(Translation3d translation) {
        return new Translation3d(16.54 - translation.getX(), translation.getY(), translation.getZ());

    }

    public boolean isAllianceRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

}