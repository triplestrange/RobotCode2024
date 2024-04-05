// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1533.frc.robot;

import com.team1533.frc.robot.Constants.JoystickButtons;
import com.team1533.frc.robot.commands.AutoMain;
import com.team1533.frc.robot.commands.DefaultDrive;
import com.team1533.frc.robot.commands.automations.AutoPickupFieldRelative;
import com.team1533.frc.robot.commands.automations.Pathfind;
import com.team1533.frc.robot.commands.automations.Shoot;
import com.team1533.frc.robot.commands.indexer.GroundToIndexer;
import com.team1533.frc.robot.commands.indexer.GroundToIntake;
import com.team1533.frc.robot.commands.indexer.IntakeToIndexer;
import com.team1533.frc.robot.subsystems.cannon.flywheel.FlyWheel;
import com.team1533.frc.robot.subsystems.cannon.flywheel.FlyWheelConstants;
import com.team1533.frc.robot.subsystems.cannon.flywheel.FlyWheelIO;
import com.team1533.frc.robot.subsystems.cannon.flywheel.FlyWheelIOReal;
import com.team1533.frc.robot.subsystems.cannon.flywheel.FlyWheelIOSim;
import com.team1533.frc.robot.subsystems.cannon.indexer.Indexer;
import com.team1533.frc.robot.subsystems.cannon.indexer.IndexerIO;
import com.team1533.frc.robot.subsystems.cannon.indexer.IndexerIOReal;
import com.team1533.frc.robot.subsystems.cannon.indexer.IndexerIOSim;
import com.team1533.frc.robot.subsystems.leds.Leds;
import com.team1533.frc.robot.subsystems.leds.Leds.LedMode;
import com.team1533.frc.robot.subsystems.rollers.Intake;
import com.team1533.frc.robot.subsystems.rollers.IntakeIO;
import com.team1533.frc.robot.subsystems.rollers.IntakeIOReal;
import com.team1533.frc.robot.subsystems.rollers.IntakeIOSim;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure.Goal;
import com.team1533.frc.robot.subsystems.superstructure.arm.Arm;
import com.team1533.frc.robot.subsystems.superstructure.arm.ArmIO;
import com.team1533.frc.robot.subsystems.superstructure.arm.ArmIOReal;
import com.team1533.frc.robot.subsystems.superstructure.arm.ArmIOSim;
import com.team1533.frc.robot.subsystems.superstructure.climb.Climber;
import com.team1533.frc.robot.subsystems.superstructure.climb.ClimberIO;
import com.team1533.frc.robot.subsystems.superstructure.climb.ClimberIOReal;
import com.team1533.frc.robot.subsystems.superstructure.climb.ClimberIOSim;
import com.team1533.frc.robot.subsystems.superstructure.elevator.Elevator;
import com.team1533.frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import com.team1533.frc.robot.subsystems.superstructure.elevator.ElevatorIOReal;
import com.team1533.frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import com.team1533.frc.robot.subsystems.swerve.GyroIO;
import com.team1533.frc.robot.subsystems.swerve.GyroIOReal;
import com.team1533.frc.robot.subsystems.swerve.GyroIOSim;
import com.team1533.frc.robot.subsystems.swerve.ModuleConstants;
import com.team1533.frc.robot.subsystems.swerve.ModuleIO;
import com.team1533.frc.robot.subsystems.swerve.ModuleIOReal;
import com.team1533.frc.robot.subsystems.swerve.ModuleIOSim;
import com.team1533.frc.robot.subsystems.swerve.SwerveConstants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;
import com.team1533.frc.robot.subsystems.vision.Vision;
import com.team1533.frc.robot.util.Alert;
import com.team1533.frc.robot.util.Alert.AlertType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        public final Robot m_robot;
        public final Superstructure m_superstructure;

        public SwerveDrive m_robotDrive;
        public Elevator m_elevator;
        public Intake m_intake;
        public Arm m_Arm;
        public FlyWheel m_flywheel;
        public Climber m_climb;
        public Indexer m_indexer;
        public final Shoot m_shoot;
        public final Vision m_vision;

        public final AutoMain m_Autos;

        public final Leds m_Leds;

        private double flywheelSetpoint;

        // The driver's controller
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(Robot m_Robot) {
                this.m_robot = m_Robot;
                m_Leds = new Leds();

                m_robotDrive = null;
                m_Arm = null;
                m_indexer = null;
                m_elevator = null;
                m_intake = null;
                m_flywheel = null;

                m_shoot = new Shoot(this);

                if (Constants.LoggerConstants.getMode() != Constants.LoggerConstants.Mode.REPLAY) {
                        switch (Constants.LoggerConstants.getRobot()) {
                                case COMPBOT -> {
                                        m_robotDrive = new SwerveDrive(this, new ModuleIOReal(ModuleConstants.FL),
                                                        new ModuleIOReal(ModuleConstants.FR),
                                                        new ModuleIOReal(ModuleConstants.BL),
                                                        new ModuleIOReal(ModuleConstants.BR),
                                                        new GyroIOReal());
                                        m_indexer = new Indexer(new IndexerIOReal());
                                        m_elevator = new Elevator(new ElevatorIOReal());
                                        m_intake = new Intake(new IntakeIOReal());
                                        m_flywheel = new FlyWheel(new FlyWheelIOReal());
                                        m_climb = new Climber(new ClimberIOReal());
                                        m_Arm = new Arm(new ArmIOReal(), m_robotDrive, m_shoot);
                                }
                                case SIMBOT -> {
                                        m_robotDrive = new SwerveDrive(this, new ModuleIOSim(ModuleConstants.FL),
                                                        new ModuleIOSim(ModuleConstants.FR),
                                                        new ModuleIOSim(ModuleConstants.BL),
                                                        new ModuleIOSim(ModuleConstants.BR),
                                                        new GyroIOSim());
                                        m_elevator = new Elevator(new ElevatorIOSim());
                                        m_intake = new Intake(new IntakeIOSim());
                                        m_indexer = new Indexer(new IndexerIOSim());
                                        m_flywheel = new FlyWheel(new FlyWheelIOSim());
                                        m_climb = new Climber(new ClimberIOSim());
                                        m_Arm = new Arm(new ArmIOSim(), m_robotDrive, m_shoot);
                                }
                        }
                }
                if (m_robotDrive == null) {
                        m_robotDrive = new SwerveDrive(this,
                                        new ModuleIO() {
                                        },
                                        new ModuleIO() {
                                        },
                                        new ModuleIO() {
                                        },
                                        new ModuleIO() {

                                        }, new GyroIO() {

                                        });
                }

                if (m_elevator == null) {
                        m_elevator = new Elevator(new ElevatorIO() {
                        });
                }
                if (m_intake == null) {
                        m_intake = new Intake(new IntakeIO() {
                        });
                }
                if (m_indexer == null) {
                        m_indexer = new Indexer(new IndexerIO() {
                        });
                }
                if (m_flywheel == null) {
                        m_flywheel = new FlyWheel(new FlyWheelIO() {
                        });
                }
                if (m_climb == null) {
                        m_climb = new Climber(new ClimberIO() {

                        });
                }
                if (m_Arm == null) {
                        m_Arm = new Arm(new ArmIO() {

                        }, m_robotDrive, m_shoot);
                }

                m_vision = new Vision(this);

                m_superstructure = new Superstructure(m_elevator, m_climb, m_Arm);
                m_Autos = new AutoMain(this);

                configureButtonBindings();

                if (Constants.LoggerConstants.tuningMode) {
                        new Alert("Tuning mode enabled", AlertType.INFO).set(true);
                }

        }

        private void configureButtonBindings() {

                // Swerve Controls

                m_robotDrive.setDefaultCommand(
                                new DefaultDrive(m_robotDrive, SwerveConstants.kMaxSpeedMetersPerSecond, 8)
                                                .withName("Normal Drive"));// 2.5,
                // 1));

                JoystickButtons.dlBump.whileTrue(
                                new DefaultDrive(m_robotDrive, 0.85, 1).withName("Slow Drive"));

                JoystickButtons.dlWing.onTrue(
                                new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive).withName("Zero Heading"));
                JoystickButtons.drWing.onTrue(
                                new InstantCommand(m_robotDrive::setXWheels, m_robotDrive).withName("Swerve Break"));

                JoystickButtons.dDpadL.onTrue(new InstantCommand(
                                () -> m_robotDrive.setHeadingController(
                                                Rotation2d.fromDegrees(m_robotDrive.isAllianceRed() ? 90 : -90)))
                                .withName("Rotate Left"));
                JoystickButtons.dDpadU.onTrue(new InstantCommand(
                                () -> m_robotDrive.setHeadingController(
                                                Rotation2d.fromDegrees(m_robotDrive.isAllianceRed() ? 0 : 180)))
                                .withName("Rotate Up"));
                JoystickButtons.dDpadR.onTrue(new InstantCommand(
                                () -> m_robotDrive.setHeadingController(
                                                Rotation2d.fromDegrees(m_robotDrive.isAllianceRed() ? -90 : 90)))
                                .withName("Rotate Right"));
                JoystickButtons.dDpadD.onTrue(new InstantCommand(
                                () -> m_robotDrive.setHeadingController(
                                                Rotation2d.fromDegrees(m_robotDrive.isAllianceRed() ? 180 : 0)))
                                .withName("Rotate Down"));

                // superstructure commands

                m_superstructure.setDefaultCommand(
                                (new RunCommand(() -> {
                                        m_elevator.moveElev(
                                                        0.3 * JoystickButtons.m_operatorController.getRightY(),
                                                        0.3 * (JoystickButtons.m_operatorController
                                                                        .getRightTriggerAxis()
                                                                        - JoystickButtons.m_operatorController
                                                                                        .getLeftTriggerAxis()));
                                        m_climb.moveClimb(
                                                        0.6 * JoystickButtons.m_operatorController.getLeftY(),
                                                        0.6 * JoystickButtons.m_operatorController.getLeftY());
                                        m_Arm.moveShooter(
                                                        0.3 * JoystickButtons.m_operatorController.getRightX());
                                }, m_superstructure)).withName("Superstructure Default Command"));

                JoystickButtons.opA.onTrue(new InstantCommand(
                                () -> m_superstructure.setGoal(Goal.STOW)).withName("Stow Superstructure"));
                JoystickButtons.opDpadU.onTrue(new InstantCommand(
                                () -> m_superstructure.setGoal(Goal.AMP)).withName("Amp Superstructure"));
                JoystickButtons.opDpadD.onTrue(new InstantCommand(
                                () -> m_superstructure.setGoal(Goal.GROUND)).withName("Ground Intake Superstructure"));

                JoystickButtons.opB.onTrue(new InstantCommand(
                                () -> m_superstructure.setGoal(Goal.CLIMB)).withName("Climb Superstructure"));

                JoystickButtons.opX.whileTrue(new InstantCommand(
                                () -> m_superstructure.setGoal(Goal.PREPARE_CLIMB))
                                .withName("Prepare Climb Superstructure"));

                // Intake and indexer Controls

                JoystickButtons.oprBump.whileTrue(new RunCommand(() -> m_intake.runIntake(),
                                m_intake)
                                .alongWith(new RunCommand(() -> m_indexer.runIn(), m_indexer)).withName("Run Intake"));

                JoystickButtons.opDpadR.whileTrue(new GroundToIntake(m_intake, m_Leds).withName("Ground To Intake"));

                JoystickButtons.opDpadL.whileTrue(
                                (new GroundToIndexer(m_indexer,m_intake ,m_Leds)).withName("Intake To Indexer"));

                JoystickButtons.oplBump.whileTrue((new RunCommand(() -> m_intake.runOutake(), m_intake)
                                .alongWith(new RunCommand(() -> m_indexer.runOut(), m_indexer))).withName("Outtake"));

                m_intake.setDefaultCommand(
                                new InstantCommand(() -> m_intake.intakeOff(), m_intake).withName("Intake Off"));

                m_indexer.setDefaultCommand(
                                new InstantCommand(() -> m_indexer.indexerOff(), m_indexer).withName("Indexer Off"));

                // Fly Wheels controls

                if (m_shoot.isAllianceRed()) {
                        flywheelSetpoint = m_robotDrive.getPose().getTranslation().getDistance(
                                        m_shoot.flipTranslation3d(m_shoot.speakerTranslation3d).toTranslation2d())
                                        * FlyWheelConstants.flyWheelmaxRPM / 3;
                } else {
                        flywheelSetpoint = m_robotDrive.getPose().getTranslation()
                                        .getDistance(m_shoot.speakerTranslation3d.toTranslation2d()) * FlyWheelConstants.flyWheelmaxRPM / 3;
                }

                flywheelSetpoint = MathUtil.clamp(flywheelSetpoint, 2500, FlyWheelConstants.flyWheelmaxRPM);

                JoystickButtons.drBump.whileTrue(new RunCommand(() -> m_flywheel.setFWSpeed(-flywheelSetpoint))
                                .withName("Manual Flywheel Speed"));

                m_flywheel.setDefaultCommand(new RunCommand(() -> m_flywheel.flyWheelOn(),
                m_flywheel));

                // Shooting Automations

                JoystickButtons.dX.whileTrue(
                                (new RunCommand(() -> m_shoot.teleopShoot(), m_indexer,
                                                m_flywheel)
                                                .alongWith(new InstantCommand(() -> m_robotDrive
                                                                .setHeadingController(
                                                                                m_shoot::rotationToSpeaker)),
                                                                new InstantCommand(
                                                                                () -> m_superstructure
                                                                                                .setGoal(Goal.AIM),
                                                                                m_superstructure)))
                                                .withName("Teleop Shoot"))
                                .onFalse((new InstantCommand(
                                                () -> m_robotDrive.setCurrentDriveMode(SwerveDrive.DriveMode.TELEOP))
                                                .alongWith(new InstantCommand(() -> m_Leds.setMode(LedMode.DEFAULT))))
                                                .withName("Teleop Shoot Off"));

                JoystickButtons.dA.whileTrue(
                                (new RunCommand(() -> m_shoot.teleopShuttle(), m_indexer, m_flywheel)
                                                .alongWith(new InstantCommand(() -> m_robotDrive
                                                                .setHeadingController(
                                                                                m_shoot::rotationToShuttle)),
                                                                new InstantCommand(
                                                                                () -> m_superstructure
                                                                                                .setGoal(Goal.SHUTTLE),
                                                                                m_superstructure)))
                                                .withName("Teleop Shuttle"))
                                .onFalse((new InstantCommand(
                                                () -> m_robotDrive.setCurrentDriveMode(SwerveDrive.DriveMode.TELEOP))
                                                .alongWith(new InstantCommand(() -> m_Leds.setMode(LedMode.DEFAULT))))
                                                .withName("Teleop Shuttle Off"));

                // Amp Automations

                // JoystickButtons.dB
                // .whileTrue(new Pathfind(Constants.FieldPositions.AMP, 0, 0, m_robotDrive));

                JoystickButtons.dB.whileTrue(
                                (new RunCommand(() -> m_robotDrive
                                                .setAutoAlignController(Constants.FieldPositions.AMP)))
                                                .withName("Drive To Amp"))
                                .onFalse((new InstantCommand(
                                                () -> m_robotDrive.setCurrentDriveMode(SwerveDrive.DriveMode.TELEOP)))
                                                .withName("Drive To Amp Off"));

                // Note Pick Automation
                // JoystickButtons.oplBump.whileTrue(new AutoPickupFieldRelative(m_robotDrive,
                // m_superstructure, m_intake,
                // m_Leds, m_vision.getObjectToField(m_robotDrive.getPose()).getTranslation())
                // .withName("Pick Up Note")).onFalse(
                // (new InstantCommand(
                // () -> m_Leds.setMode(LedMode.DEFAULT)))
                // .withName("Note Pick Up Off"));

                // leds
                        m_Leds.setDefaultCommand(new InstantCommand(() -> m_Leds.setMode(LedMode.DEFAULT), m_Leds));

        }

        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

}