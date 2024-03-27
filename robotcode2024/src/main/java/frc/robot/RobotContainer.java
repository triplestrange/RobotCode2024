// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickButtons;
import frc.robot.commands.AutoMain;
import frc.robot.subsystems.cannon.climb.Climb;
import frc.robot.subsystems.cannon.flywheel.FlyWheel;
import frc.robot.subsystems.cannon.flywheel.FlyWheelIO;
import frc.robot.subsystems.cannon.flywheel.FlyWheelIOReal;
import frc.robot.subsystems.cannon.flywheel.FlyWheelIOSim;
import frc.robot.subsystems.cannon.indexer.Indexer;
import frc.robot.subsystems.cannon.indexer.IndexerIO;
import frc.robot.subsystems.cannon.indexer.IndexerIOReal;
import frc.robot.subsystems.cannon.indexer.IndexerIOSim;
import frc.robot.subsystems.cannon.shooter.Shooter;
import frc.robot.subsystems.intake.elevator.Elevator;
import frc.robot.subsystems.intake.elevator.ElevatorIO;
import frc.robot.subsystems.intake.elevator.ElevatorIOReal;
import frc.robot.subsystems.intake.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.rollers.Intake;
import frc.robot.subsystems.intake.rollers.IntakeIO;
import frc.robot.subsystems.intake.rollers.IntakeIOReal;
import frc.robot.subsystems.intake.rollers.IntakeIOSim;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.automations.AutoPickupFieldRelative;
import frc.robot.commands.automations.DriveTo;
import frc.robot.commands.automations.Shoot;
import frc.robot.commands.indexer.GroundToIndexer;
import frc.robot.commands.indexer.GroundToIntake;
import frc.robot.commands.indexer.IntakeToIndexer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        public final Robot m_robot;
        public final SwerveDrive m_robotDrive;
        public Elevator m_elevator;
        public Intake m_intake;
        public final Shooter m_shooter;
        public FlyWheel m_flywheel;
        public final Climb m_climb;
        public Indexer m_indexer;
        public final Shoot m_shoot;
        public final Vision m_vision;

        // private final SendableChooser<Command> choose;
        public final AutoMain m_Autos;

        private double flywheelSetpoint;

        // The driver's controller
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(Robot m_Robot) {
                this.m_robot = m_Robot;
                m_robotDrive = new SwerveDrive(this);
                m_shooter = new Shooter(m_robotDrive);
                m_climb = new Climb();
                m_shoot = new Shoot(this);

                m_indexer = null;
                m_elevator = null;
                m_intake = null;
                m_flywheel = null;

                if (Constants.LoggerConstants.getMode() != Constants.LoggerConstants.Mode.REPLAY) {
                        switch (Constants.LoggerConstants.getRobot()) {
                                case COMPBOT -> {
                                        m_indexer = new Indexer(new IndexerIOReal());
                                        m_elevator = new Elevator(new ElevatorIOReal());
                                        m_intake = new Intake(new IntakeIOReal());
                                        m_flywheel = new FlyWheel(new FlyWheelIOReal());

                                }
                                case SIMBOT -> {
                                        m_elevator = new Elevator(new ElevatorIOSim());
                                        m_intake = new Intake(new IntakeIOSim());
                                        m_indexer = new Indexer(new IndexerIOSim());
                                        m_flywheel = new FlyWheel(new FlyWheelIOSim());

                                }
                        }
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
                m_vision = new Vision(this.m_robotDrive, this.m_shoot, this.m_elevator);

                m_Autos = new AutoMain(this);

                configureButtonBindings();

                if (Constants.LoggerConstants.tuningMode) {
                        new Alert("Tuning mode enabled", AlertType.INFO).set(true);
                }

        }

        private void configureButtonBindings() {

                // Swerve Controls

                m_robotDrive.setDefaultCommand(
                                new DefaultDrive(m_robotDrive, 4.7, 2));// 2.5, 1));

                JoystickButtons.dlBump.whileTrue(
                                new DefaultDrive(m_robotDrive, 0.85, 1));

                JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
                JoystickButtons.drWing.onTrue(new InstantCommand(m_robotDrive::setXWheels, m_robotDrive));

                JoystickButtons.dDpadL.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? -90 : 90)));
                JoystickButtons.dDpadU.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? 0 : 180)));
                JoystickButtons.dDpadR.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? 90 : -90)));
                JoystickButtons.dDpadD.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? 180 : 0)));

                // Elevator Controls

                JoystickButtons.opA.onTrue(new InstantCommand(
                                () -> m_elevator.setElev(Constants.MechPositions.stowIntakePos), m_elevator));
                JoystickButtons.opDpadU.onTrue(new InstantCommand(
                                () -> m_elevator.setElev(Constants.MechPositions.ampIntakePos), m_elevator));
                JoystickButtons.opDpadD.onTrue(new InstantCommand(
                                () -> m_elevator.setElev(Constants.MechPositions.groundIntakePos)));

                m_elevator.setDefaultCommand(new RunCommand(
                                () -> m_elevator.moveElev(
                                                -1 * JoystickButtons.m_operatorController.getRightY(),
                                                0.3 * JoystickButtons.m_operatorController.getRightX()),
                                m_elevator));

                // Climb Controls

                // m_climb.setDefaultCommand(new RunCommand(
                // () -> m_climb.moveClimb(
                // 0.6 * JoystickButtons.m_operatorController.getLeftY(),
                // 0.6 * JoystickButtons.m_operatorController.getLeftY()),
                // m_climb));

                // Pivot Controls

                JoystickButtons.opB.onTrue(new InstantCommand(
                                () -> m_shooter.setShooterAngle(Constants.MechPositions.climbPivotPos)));

                JoystickButtons.opX.whileTrue(new InstantCommand(
                                () -> m_shooter.setShooterAngle(Constants.MechPositions.clearancePivotPos)));
                JoystickButtons.opY.whileTrue(new InstantCommand(
                                () -> m_shooter.setShooterAngle(Constants.MechPositions.lowPivotPos)));

                m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.moveShooter(
                                0.25 * JoystickButtons.m_operatorController.getLeftY()), m_shooter));
                // Intake and indexer Controls

                JoystickButtons.oprBump.whileTrue(new RunCommand(() -> m_intake.runIntake(),
                m_intake)
                .alongWith(new RunCommand(() -> m_indexer.runIn(), m_indexer)));

                JoystickButtons.opDpadR.whileTrue(new GroundToIntake(m_intake));

                JoystickButtons.opDpadL.whileTrue(
                                (new IntakeToIndexer(m_indexer)));

                JoystickButtons.oplBump.whileTrue(new RunCommand(() -> m_intake.runOutake(), m_intake)
                                .alongWith(new RunCommand(() -> m_indexer.runOut(), m_indexer)));

                m_intake.setDefaultCommand(new InstantCommand(() -> m_intake.intakeOff(), m_intake));

                m_indexer.setDefaultCommand(new InstantCommand(() -> m_indexer.indexerOff(), m_indexer));

                // Fly Wheels controls

                if (m_shoot.isAllianceRed()) {
                        flywheelSetpoint = m_robotDrive.getPose().getTranslation().getDistance(
                                        m_shoot.flipTranslation3d(m_shoot.speakerTranslation3d).toTranslation2d())
                                        * 4850.0 / 3;
                } else {
                        flywheelSetpoint = m_robotDrive.getPose().getTranslation()
                                        .getDistance(m_shoot.speakerTranslation3d.toTranslation2d()) * 4850.0 / 6;
                }

                flywheelSetpoint = MathUtil.clamp(flywheelSetpoint, 2500, 4850);

                JoystickButtons.drBump.whileTrue(new RunCommand(() -> m_flywheel.setFWSpeed(-flywheelSetpoint)));

                m_flywheel.setDefaultCommand(new RunCommand(() -> m_flywheel.flyWheelOff(), m_flywheel));

                // Shooting Automations

                JoystickButtons.dX.whileTrue(
                                new RunCommand(() -> m_shoot.autoShoot(), m_shooter, m_indexer));
                // Amp Automations

                JoystickButtons.dB
                                .whileTrue(new DriveTo(Constants.MechPositions.amp, 0, 0, m_robotDrive));

                // Note Pick Automation
                // JoystickButtons.oplBump.whileTrue(new AutoPickupFieldRelative(m_robotDrive, m_elevator, m_intake,
                //                 m_vision.getObjectToField(m_robotDrive.getPose())));
        }

        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

}