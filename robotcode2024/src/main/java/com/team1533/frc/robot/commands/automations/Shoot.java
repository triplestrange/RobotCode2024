// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD license file in the root directory of this project. 

package com.team1533.frc.robot.commands.automations;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;
import javax.swing.text.GlyphView.GlyphPainter;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.Robot;
import com.team1533.frc.robot.RobotContainer;
import com.team1533.frc.robot.Constants.JoystickButtons;
import com.team1533.frc.robot.Constants.SwerveConstants;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure.Goal;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;
import com.team1533.lib.control.HeadingController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot {

    /**
     * Creates a new Approach.
     * Robot will drive to specified point (parameter)
     **/

    RobotContainer m_RobotContainer;

    private ProfiledPIDController rotation_controller;

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
    double flywheelSetpoint;
    public Translation3d speakerTranslation3d = new Translation3d(0, 5.6282082, 2 + 0.035);

    public InterpolatingDoubleTreeMap shootingData = new InterpolatingDoubleTreeMap();

    double speedR;
    double rot;

    public Shoot(RobotContainer m_RobotContainer) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_RobotContainer = m_RobotContainer;
        shootingRotation = m_RobotContainer.m_robotDrive.getPose().getRotation();

        // driveTo = new DriveTo(new Pose2d(
        // speakerTranslation3d.getX() + Units.inchesToMeters(16) +
        // Units.inchesToMeters(36.241382),
        // speakerTranslation3d.getY(), new Rotation2d().fromDegrees(0)), 0, 0,
        // m_RobotContainer.m_robotDrive,
        // m_RobotContainer.m_robot);

        shootingData.put(1.0, 0.0);
        shootingData.put(1.655738, -12.2);
        shootingData.put(2.2, -16.0);
        shootingData.put(3.120114, -23.5);
        shootingData.put(4.991135, -31.55);
        shootingData.put(5.3, -31.0);
        shootingData.put(6.38, -33.4);
        shootingData.put(7.39, -34.5);

        rotation_controller = new ProfiledPIDController(0.2, 0.17, 0.015, new Constraints(720, 100));
        rotation_controller.enableContinuousInput(0, 360);
        rotation_controller.setIZone(10);

        // System.out.println("rotation PID: " + rot);

        // System.out.println("gyro: " + m_swerve.getAngle().getDegrees());
        // System.out.println("desiredHeading: " + m_swerve.getRotationPreset());

    }

    public void prepare() {
        if (isAllianceRed()) {
            shootingRotation = new Translation2d(flipTranslation3d(speakerTranslation3d).getX(),
                    flipTranslation3d(speakerTranslation3d).getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(new Rotation2d().fromDegrees(180));
            // .plus(new Rotation2d(getRelativeHorizontalSpeedMetersPerSecond(
            // m_RobotContainer.m_robotDrive.getChassisSpeeds(),
            // m_RobotContainer.m_robotDrive.getPose())
            // * m_RobotContainer.m_robotDrive.getPose().getTranslation()
            // .getDistance((speakerTranslation3d.toTranslation2d()))
            // * -0.1));
        } else {
            shootingRotation = new Translation2d(speakerTranslation3d.getX(), speakerTranslation3d.getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(new Rotation2d().fromDegrees(180));
            // .plus(new Rotation2d(getRelativeHorizontalSpeedMetersPerSecond(
            // m_RobotContainer.m_robotDrive.getChassisSpeeds(),
            // m_RobotContainer.m_robotDrive.getPose())
            // * m_RobotContainer.m_robotDrive.getPose().getTranslation()
            // .getDistance((speakerTranslation3d.toTranslation2d()))
            // * -0.1));
        }

        m_RobotContainer.m_robotDrive.setPresetEnabled(true, shootingRotation.getDegrees());
        if (isAllianceRed()) {
            flywheelSetpoint = m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance(flipTranslation3d(speakerTranslation3d).toTranslation2d()) * 4850.0 / 3;
        } else {
            flywheelSetpoint = m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance(speakerTranslation3d.toTranslation2d()) * 4850.0 / 3;
        }

        flywheelSetpoint = MathUtil.clamp(flywheelSetpoint, 2500, 4850);

        m_RobotContainer.m_flywheel.setFWSpeed(-flywheelSetpoint);

    }

    public void autoPrepare() {
        if (isAllianceRed()) {
            shootingRotation = new Translation2d(flipTranslation3d(speakerTranslation3d).getX(),
                    flipTranslation3d(speakerTranslation3d).getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(new Rotation2d().fromDegrees(180));
        } else {
            shootingRotation = new Translation2d(speakerTranslation3d.getX(), speakerTranslation3d.getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(new Rotation2d().fromDegrees(180));
        }
        rot = rotation_controller.calculate(m_RobotContainer.m_robotDrive.getPose().getRotation().getDegrees(),
                shootingRotation.getDegrees());
        rot = MathUtil.clamp(rot, -2 * Math.PI, 2 * Math.PI);

        speedR = rot;

        m_RobotContainer.m_robotDrive.drive(
                0,
                0,
                speedR,
                true);

        if (isAllianceRed()) {
            shootingAngle = shootingData.get(m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance(flipTranslation3d(speakerTranslation3d).toTranslation2d()));
        } else {
            shootingAngle = shootingData.get(m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance((speakerTranslation3d.toTranslation2d())));
        }

    }

    public void execute() {

        m_RobotContainer.m_superstructure.setGoalCommand(Goal.AIM);
    }

    public void shoot() {
        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose()) && pivotCheck() && flyWheelCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose()) && velocityCheck()) {

            m_RobotContainer.m_indexer.runIn();

            m_RobotContainer.m_robotDrive.setPresetEnabled(true);

        }
    }

    public void autonomousShoot() {
        if (pivotCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose())
                && velocityCheck()) {

            m_RobotContainer.m_indexer.runIn();

            m_RobotContainer.m_robotDrive.setPresetEnabled(false);

        }
    }

    public void autoShoot() {
        prepare();

        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose())) {
            // execute();
            shoot();
        }

    }

    public Rotation2d rotationToSpeaker() {
        if (isAllianceRed()) {
            shootingRotation = new Translation2d(flipTranslation3d(speakerTranslation3d).getX(),
                    flipTranslation3d(speakerTranslation3d).getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(new Rotation2d().fromDegrees(180));
        } else {
            shootingRotation = new Translation2d(speakerTranslation3d.getX(), speakerTranslation3d.getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(new Rotation2d().fromDegrees(180));

        }

        return shootingRotation;

    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        // Some condition that should decide if we want to override rotation
        if (m_RobotContainer.m_indexer.getIndexerSensor()) {
            // Return an optional containing the rotation override (this should be a field
            // relative rotation)
            return Optional.of(rotationToSpeaker());
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }

    public void autonomous() {
        autoPrepare();
        execute();
        autonomousShoot();

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

    public Boolean velocityCheck() {
        return m_RobotContainer.m_robotDrive.isMovingXY();
    }

    public Boolean pivotCheck() {
        return Math.abs(m_RobotContainer.m_Arm.getAngle() - m_RobotContainer.m_Arm.shootingAngle) < .5;
    }

    public Boolean rotationCheck(Pose2d robotPose2d) {
        return robotPose2d.getRotation().minus(shootingRotation).getDegrees() < 3;
    }

    public Boolean flyWheelCheck() {
        return Math.abs(m_RobotContainer.m_flywheel.getLeftSpeed() - (-flywheelSetpoint)) < 300;
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

    public double getRelativeVerticalSpeedMetersPerSecond(ChassisSpeeds robotSpeeds, Pose2d robotPose) {
        Rotation2d robotRot = getSpeakerForCurrentAlliance().toTranslation2d().minus(robotPose.getTranslation())
                .getAngle();
        Translation2d fieldOrientSpeeds = new Translation2d(robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation()).rotateBy(robotRot.unaryMinus());
        return fieldOrientSpeeds.getX();
    }

    public double getRelativeHorizontalSpeedMetersPerSecond(ChassisSpeeds robotSpeeds, Pose2d robotPose) {
        Rotation2d robotRot = getSpeakerForCurrentAlliance().toTranslation2d().minus(robotPose.getTranslation())
                .getAngle();
        Translation2d fieldOrientSpeeds = new Translation2d(robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation()).rotateBy(robotRot.unaryMinus());
        return fieldOrientSpeeds.getY();
    }

    public Translation3d getSpeakerForCurrentAlliance() {
        if (isAllianceRed()) {
            return flipTranslation3d(speakerTranslation3d);
        } else {
            return speakerTranslation3d;
        }
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("swerve check ", swerveCheck(m_RobotContainer.m_robotDrive.getPose()));
        SmartDashboard.putBoolean("roation check", rotationCheck(m_RobotContainer.m_robotDrive.getPose()));
        SmartDashboard.putBoolean("pivot check", pivotCheck());
        SmartDashboard.putBoolean("flywheel check", flyWheelCheck());
        SmartDashboard.putBoolean("Velocity Check", velocityCheck());
        SmartDashboard.putBoolean("Red Alliance?", isAllianceRed());

        SmartDashboard.putNumber("target x", flipTranslation3d(speakerTranslation3d).getX());
        SmartDashboard.putNumber("target y", flipTranslation3d(speakerTranslation3d).getY());

        SmartDashboard.putNumber("target z", flipTranslation3d(speakerTranslation3d).getZ());

        SmartDashboard.putNumber("normalized distance", m_RobotContainer.m_robotDrive.getPose().getTranslation()
                .getDistance(speakerTranslation3d.toTranslation2d()));

    }
}