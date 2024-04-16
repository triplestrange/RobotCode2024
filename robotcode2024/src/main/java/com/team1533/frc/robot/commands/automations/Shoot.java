// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD license file in the root directory of this project. 

package com.team1533.frc.robot.commands.automations;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team1533.frc.robot.RobotContainer;
import com.team1533.frc.robot.subsystems.cannon.flywheel.FlyWheelConstants;
import com.team1533.frc.robot.subsystems.leds.Leds.LedMode;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure.Goal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot {

    /**
     * Creates a new Approach.
     * Robot will drive to specified point (parameter)
     **/

    RobotContainer m_RobotContainer;

    // private ProfiledPIDController rotation_controller;

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

    


    // variables for shooting
    @AutoLogOutput(key = "Shoot/shootingRotation")
    Rotation2d shootingRotation;
    @AutoLogOutput(key = "Shoot/shootingAngle")
    double shootingAngle = 0;
    @AutoLogOutput(key = "Shoot/flywheelSetpoint")
    double flywheelSetpoint;
    public Translation3d speakerTranslation3d = new Translation3d(0, 5.6282082, 2 + 0.035);

    public Translation2d shuttlingTranslation2d = new Translation2d(1.14, 7.11);

    public InterpolatingDoubleTreeMap shootingData = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap shuttlingData = new InterpolatingDoubleTreeMap();

    // double speedR;
    // double rot;

    private Debouncer pDebouncer = new Debouncer(0.1);
    private Debouncer rDebouncer = new Debouncer(0.1);
    private boolean pivotCheck = false;
    private boolean canShoot = false;
    private boolean rotationCheck = false;

    double compensationHorizontal = 0.0125;
    double compensationVertical = 0.35;

    public Shoot(RobotContainer m_RobotContainer) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_RobotContainer = m_RobotContainer;
        shootingRotation = new Rotation2d();

        shootingData.put(0.0, 0.0);
        shootingData.put(1.0, 0.0);
        shootingData.put(1.5, -3.2);
        shootingData.put(2.0, -9.5);
        shootingData.put(2.5, -16.5);
        shootingData.put(3.0, -23.5);

        shootingData.put(4.0, -30.6);

        shootingData.put(5.0, -31.5);
        shootingData.put(6.0, -33.5);
        shootingData.put(7.0, -36.0);
        shootingData.put(8.0, -37.5);


                
        shuttlingData.put(7.0, 1533.0);
        shuttlingData.put(8.0, 1800.0);
        shuttlingData.put(9.0, 2000.0);
        shuttlingData.put(10.0, 2100.0);
        shuttlingData.put(11.0, 2150.0);
        

    }

    public void teleopShoot() {
        if (isAllianceRed()) {
            flywheelSetpoint = m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance(flipTranslation3d(speakerTranslation3d).toTranslation2d())
                    * FlyWheelConstants.flyWheelmaxRPM;
        } else {
            flywheelSetpoint = m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance(speakerTranslation3d.toTranslation2d()) * FlyWheelConstants.flyWheelmaxRPM;
        }

        flywheelSetpoint = MathUtil.clamp(flywheelSetpoint, 2500, FlyWheelConstants.flyWheelmaxRPM);

        m_RobotContainer.m_flywheel.setFWSpeed(-flywheelSetpoint);
        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose()) && pivotShootCheck() && flyWheelCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose()) && velocityCheckTeleop()) {

            m_RobotContainer.m_indexer.runIn();

        }
        if (!swerveCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.SWERVE_CHECK);
        } else if (!velocityCheckTeleop()) {
            m_RobotContainer.m_Leds.setMode(LedMode.VELOCITY_CHECK);
        } else if (!pivotShootCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.PIVOT_CHECK);

        } else if (!rotationCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.ROTATION_CHECK);

        } else if (!flyWheelCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.FLYWHEEL_CHECK);

        }

        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose()) && velocityCheckTeleop() && pivotShootCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose()) && flyWheelCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.AUTO_SHOOT);
        }

    }

    public void teleopShuttle() {
        flywheelSetpoint = shuttlingData.get(isAllianceRed() ? m_RobotContainer.m_robotDrive.getPose().getTranslation()
                .getDistance(flipTranslation2d(
                        shuttlingTranslation2d))
                : m_RobotContainer.m_robotDrive.getPose().getTranslation()
                        .getDistance(shuttlingTranslation2d));

        flywheelSetpoint = MathUtil.clamp(flywheelSetpoint, 0, FlyWheelConstants.flyWheelmaxRPM);

        m_RobotContainer.m_flywheel.setFWSpeed(-flywheelSetpoint, 0);

        if (flyWheelCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose()) && velocityCheckTeleop() && pivotShuttleCheck() && shuttleCheck(m_RobotContainer.m_robotDrive.getPose())) {

            m_RobotContainer.m_indexer.runIn();

        }

        if (!shuttleCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.SWERVE_CHECK);
        } else if (!velocityCheckTeleop()) {
            m_RobotContainer.m_Leds.setMode(LedMode.VELOCITY_CHECK);
        } else if (!pivotShuttleCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.PIVOT_CHECK);

        } else if (!rotationCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.ROTATION_CHECK);

        } else if (!flyWheelCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.FLYWHEEL_CHECK);

        }

    }

    public void autoShoot() {
        if (pivotShootCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose())
                && velocityCheckAuto()) {

            m_RobotContainer.m_indexer.runIn();

        }

        if (!swerveCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.SWERVE_CHECK);
        } else if (!velocityCheckAuto()) {
            m_RobotContainer.m_Leds.setMode(LedMode.VELOCITY_CHECK);
        } else if (!pivotShootCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.PIVOT_CHECK);

        } else if (!rotationCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.ROTATION_CHECK);

        } else if (!flyWheelCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.FLYWHEEL_CHECK);

        }

        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose()) && velocityCheckAuto() && pivotShootCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose()) && flyWheelCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.AUTO_SHOOT);
        }

    }

    public void autoShootMove() {

        m_RobotContainer.m_indexer.runIn();

        if (!swerveCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.SWERVE_CHECK);
        } else if (!velocityCheckTeleop()) {
            m_RobotContainer.m_Leds.setMode(LedMode.VELOCITY_CHECK);
        } else if (!pivotShootCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.PIVOT_CHECK);

        } else if (!rotationCheck(m_RobotContainer.m_robotDrive.getPose())) {
            m_RobotContainer.m_Leds.setMode(LedMode.ROTATION_CHECK);

        } else if (!flyWheelCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.FLYWHEEL_CHECK);

        }

        if (swerveCheck(m_RobotContainer.m_robotDrive.getPose()) && velocityCheckTeleop() && pivotShootCheck()
                && rotationCheck(m_RobotContainer.m_robotDrive.getPose()) && flyWheelCheck()) {
            m_RobotContainer.m_Leds.setMode(LedMode.AUTO_SHOOT);
        }

    }
 @AutoLogOutput
    public Rotation2d rotationToSpeaker() {
        if (isAllianceRed()) {
            shootingRotation = new Translation2d(flipTranslation3d(speakerTranslation3d).getX(),
                    flipTranslation3d(speakerTranslation3d).getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(Rotation2d.fromDegrees(180))
                    .minus(new Rotation2d(getRelativeHorizontalSpeedMetersPerSecond(
                            m_RobotContainer.m_robotDrive.getChassisSpeeds(),
                            m_RobotContainer.m_robotDrive.getPose())
                            * m_RobotContainer.m_robotDrive.getPose().getTranslation()
                                    .getDistance((speakerTranslation3d.toTranslation2d()))
                            * compensationHorizontal));
        } else {
            shootingRotation = new Translation2d(speakerTranslation3d.getX(), speakerTranslation3d.getY())
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(Rotation2d.fromDegrees(180))
                    .minus(new Rotation2d(getRelativeHorizontalSpeedMetersPerSecond(
                            m_RobotContainer.m_robotDrive.getChassisSpeeds(),
                            m_RobotContainer.m_robotDrive.getPose())
                            * m_RobotContainer.m_robotDrive.getPose().getTranslation()
                                    .getDistance((speakerTranslation3d.toTranslation2d()))
                            * compensationHorizontal));

        }

        return shootingRotation;

    }
 @AutoLogOutput
    public Rotation2d rotationToShuttle() {
        if (isAllianceRed()) {
            shootingRotation = flipTranslation2d(shuttlingTranslation2d)
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(Rotation2d.fromDegrees(180))
                    .minus(new Rotation2d(getRelativeHorizontalSpeedMetersPerSecond(
                            m_RobotContainer.m_robotDrive.getChassisSpeeds(),
                            m_RobotContainer.m_robotDrive.getPose())
                            * m_RobotContainer.m_robotDrive.getPose().getTranslation()
                                    .getDistance((speakerTranslation3d.toTranslation2d()))
                            * compensationHorizontal));
        } else {
            shootingRotation = shuttlingTranslation2d
                    .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()).getAngle()
                    .plus(Rotation2d.fromDegrees(180))
                    .minus(new Rotation2d(getRelativeHorizontalSpeedMetersPerSecond(
                            m_RobotContainer.m_robotDrive.getChassisSpeeds(),
                            m_RobotContainer.m_robotDrive.getPose())
                            * m_RobotContainer.m_robotDrive.getPose().getTranslation()
                                    .getDistance((speakerTranslation3d.toTranslation2d()))
                            * compensationHorizontal));

        }

        return shootingRotation;

    }
 @AutoLogOutput
    public double degToSpeaker() {
        if (isAllianceRed()) {
            shootingAngle = shootingData.get(m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance(flipTranslation3d(speakerTranslation3d).toTranslation2d())
                    - getRelativeVerticalSpeedMetersPerSecond(
                            m_RobotContainer.m_robotDrive.getChassisSpeeds(),
                            m_RobotContainer.m_robotDrive.getPose())
                            * compensationVertical);
        } else {
            shootingAngle = shootingData.get(m_RobotContainer.m_robotDrive.getPose().getTranslation()
                    .getDistance((speakerTranslation3d.toTranslation2d()))
                    - getRelativeVerticalSpeedMetersPerSecond(
                            m_RobotContainer.m_robotDrive.getChassisSpeeds(),
                            m_RobotContainer.m_robotDrive.getPose())
                            * compensationVertical);
        }
        return shootingAngle;
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
//  @AutoLogOutput
    public Boolean shuttleCheck(Pose2d robotPose2d) {
          if (isAllianceRed()) {
            robotPose2dInches = new Translation2d(Units.metersToInches(flipPose(robotPose2d).getX()),
                    Units.metersToInches(flipPose(robotPose2d).getY()));
        } else {
            robotPose2dInches = new Translation2d(Units.metersToInches(robotPose2d.getX()),
                    Units.metersToInches(robotPose2d.getY()));
        }
        if (robotPose2dInches.getX() > Units.metersToInches(7)) {
            return true;

    }
    return false;
    }
    //  @AutoLogOutput
    public Boolean swerveCheck(Pose2d robotPose2d) {

        if (isAllianceRed()) {
            robotPose2dInches = new Translation2d(Units.metersToInches(flipPose(robotPose2d).getX()),
                    Units.metersToInches(flipPose(robotPose2d).getY()));
        } else {
            robotPose2dInches = new Translation2d(Units.metersToInches(robotPose2d.getX()),
                    Units.metersToInches(robotPose2d.getY()));
        }
        if (robotPose2dInches.getX() > 0) {
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

        if (canShoot) {
        }
        return true;
    }
 @AutoLogOutput
    public Boolean velocityCheckTeleop() {
        boolean isSlowEnough = m_RobotContainer.m_robotDrive.isMovingXY();
        if (isSlowEnough) {
        }
        return isSlowEnough;
    }
 @AutoLogOutput
    public Boolean velocityCheckAuto()  {
         boolean isSlowEnough = m_RobotContainer.m_robotDrive.isMovingXYAuto();
        if (isSlowEnough) {
        }
        return isSlowEnough;
    }
 @AutoLogOutput
    public Boolean pivotShootCheck() {
        if (Math.hypot(m_RobotContainer.m_robotDrive.getChassisSpeeds().vxMetersPerSecond, m_RobotContainer.m_robotDrive.getChassisSpeeds().vyMetersPerSecond) > 1) {
            pivotCheck = Math.abs(m_RobotContainer.m_Arm.getAngle() - m_RobotContainer.m_Arm.shootingAngle) < 2;
        }   else {
        
        pivotCheck = Math.abs(m_RobotContainer.m_Arm.getAngle() - m_RobotContainer.m_Arm.shootingAngle) < .5;
        }
        return pDebouncer.calculate(pivotCheck);
    }
 @AutoLogOutput
    public Boolean pivotShuttleCheck()  {
        return Math.abs(m_RobotContainer.m_Arm.getAngle() + 12.5) < 0.5;
    }
//  @AutoLogOutput
    public Boolean rotationCheck(Pose2d robotPose2d) {
                if (Math.hypot(m_RobotContainer.m_robotDrive.getChassisSpeeds().vxMetersPerSecond, m_RobotContainer.m_robotDrive.getChassisSpeeds().vyMetersPerSecond) > 1) {

      rotationCheck  = robotPose2d.getRotation().minus(shootingRotation).getDegrees() < 3;
               
    } else {

        rotationCheck = robotPose2d.getRotation().minus(shootingRotation).getDegrees() < 2;

    }
        return rDebouncer.calculate(rotationCheck);

}
 @AutoLogOutput
    public Boolean flyWheelCheck() {
        boolean flyWheelCheck = Math.abs(m_RobotContainer.m_flywheel.getLeftSpeed() - (-flywheelSetpoint)) < 100;
        if (flyWheelCheck) {

        }
        return flyWheelCheck;
    }
//  @AutoLogOutput
    public Pose2d flipPose(Pose2d pose) {
        return new Pose2d(16.54 - pose.getX(), pose.getY(),
                pose.getRotation().rotateBy(new Rotation2d().fromRadians(Math.PI / 2)));
    }
//  @AutoLogOutput
    public Translation2d flipTranslation2d(Translation2d translation) {
        return new Translation2d(16.54 - translation.getX(), translation.getY());
    }
//  @AutoLogOutput
    public Translation3d flipTranslation3d(Translation3d translation) {
        return new Translation3d(16.54 - translation.getX(), translation.getY(), translation.getZ());

    }
 @AutoLogOutput
    public boolean isAllianceRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
//  @AutoLogOutput
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

    public void periodic() {

    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("swerve check ", swerveCheck(m_RobotContainer.m_robotDrive.getPose()));
        SmartDashboard.putBoolean("roation check", rotationCheck(m_RobotContainer.m_robotDrive.getPose()));
        SmartDashboard.putBoolean("pivot check", pivotShuttleCheck());
        SmartDashboard.putBoolean("flywheel check", flyWheelCheck());
        SmartDashboard.putBoolean("Velocity Check", velocityCheckTeleop());
        SmartDashboard.putBoolean("Red Alliance?", isAllianceRed());

        SmartDashboard.putNumber("target x", flipTranslation3d(speakerTranslation3d).getX());
        SmartDashboard.putNumber("target y", flipTranslation3d(speakerTranslation3d).getY());

        SmartDashboard.putNumber("target z", flipTranslation3d(speakerTranslation3d).getZ());

        SmartDashboard.putNumber("normalized distance", m_RobotContainer.m_robotDrive.getPose().getTranslation()
                .getDistance(shuttlingTranslation2d));

        SmartDashboard.putNumber("relativeH", getRelativeHorizontalSpeedMetersPerSecond(
                m_RobotContainer.m_robotDrive.getChassisSpeeds(), m_RobotContainer.m_robotDrive.getPose()));
        SmartDashboard.putNumber("relativeV", getRelativeVerticalSpeedMetersPerSecond(
                m_RobotContainer.m_robotDrive.getChassisSpeeds(), m_RobotContainer.m_robotDrive.getPose()));

    }
}