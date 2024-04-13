package com.team1533.frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team1533.frc.robot.RobotContainer;
import com.team1533.frc.robot.commands.indexer.IntakeToIndexer;
import com.team1533.frc.robot.subsystems.cannon.flywheel.FlyWheelConstants;
import com.team1533.frc.robot.subsystems.leds.Leds.LedMode;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure.Goal;
import com.team1533.frc.robot.subsystems.superstructure.arm.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoMain extends Command {

        RobotContainer m_robotContainer;
        private final SendableChooser<Command> autoChooser;

        public AutoMain(RobotContainer m_robotContainer) {
                // Class Variables
                this.m_robotContainer = m_robotContainer;

                registerCommands();

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        public Optional<Rotation2d> getRotationTargetOverride() {
                return Optional.of(m_robotContainer.m_shoot.rotationToSpeaker());
        }

        public void registerCommands() {
                // Elev Commands
                NamedCommands.registerCommand("amp",
                                new InstantCommand(() -> m_robotContainer.m_superstructure.setGoal(
                                                Superstructure.Goal.AMP),
                                                m_robotContainer.m_superstructure));
                NamedCommands.registerCommand("ground", new InstantCommand(
                                () -> m_robotContainer.m_superstructure.setGoal(Superstructure.Goal.GROUND_AUTO),
                                m_robotContainer.m_superstructure));

                NamedCommands.registerCommand("stowElev",
                                new InstantCommand(() -> m_robotContainer.m_superstructure.setGoal(
                                                Superstructure.Goal.STOW),
                                                m_robotContainer.m_superstructure));
                // Intake Commands
                NamedCommands.registerCommand("intakeOut", new InstantCommand(() -> m_robotContainer.m_intake
                                .runOutake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeIn", new InstantCommand(() -> m_robotContainer.m_intake
                                .runIntake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> m_robotContainer.m_intake
                                .intakeOff(), m_robotContainer.m_intake));

                // Conveyor Commands
                NamedCommands.registerCommand("indexerOut", new InstantCommand(
                                () -> m_robotContainer.m_indexer.runOut(), m_robotContainer.m_indexer));

                NamedCommands.registerCommand("conveyorIn",
                                new IntakeToIndexer(m_robotContainer.m_indexer, m_robotContainer.m_Leds));

                NamedCommands.registerCommand("indexerIn", new IntakeToIndexer(m_robotContainer.m_indexer,
                                m_robotContainer.m_Leds));

                NamedCommands.registerCommand("indexerShoot", new InstantCommand(
                                () -> m_robotContainer.m_indexer.runIn(), m_robotContainer.m_indexer));
                NamedCommands.registerCommand("conveyorShoot", new InstantCommand(
                                () -> m_robotContainer.m_indexer.runIn(), m_robotContainer.m_indexer));

                NamedCommands.registerCommand("indexerOff", new InstantCommand(
                                () -> m_robotContainer.m_indexer.indexerOff(), m_robotContainer.m_indexer));

                NamedCommands.registerCommand("intakeNote",
                                (new ParallelDeadlineGroup(new IntakeToIndexer(m_robotContainer.m_indexer,
                                                m_robotContainer.m_Leds))
                                                .alongWith(
                                                                new RunCommand(
                                                                                () -> m_robotContainer.m_intake
                                                                                                .runIntake(),
                                                                                m_robotContainer.m_intake))));

                // Shooter
                NamedCommands.registerCommand("shoot",
                                (new RunCommand(() -> m_robotContainer.m_shoot.autoShoot(),
                                                m_robotContainer.m_indexer)
                                                .alongWith(new InstantCommand(() -> m_robotContainer.m_robotDrive
                                                                .setHeadingControllerInAuto(
                                                                                m_robotContainer.m_shoot::rotationToSpeaker))
                                                                .alongWith(new InstantCommand(
                                                                                () -> m_robotContainer.m_robotDrive
                                                                                                .acceptTeleopInput(0, 0,
                                                                                                                0,
                                                                                                                true)))
                ).withTimeout(1.25)));

                NamedCommands.registerCommand("shoot move", new RunCommand(
                                () -> m_robotContainer.m_shoot.autoShootMove(), m_robotContainer.m_indexer));

                NamedCommands.registerCommand("shoot fixed", new InstantCommand(() -> m_robotContainer.m_flywheel.setFWSpeed(-FlyWheelConstants.flyWheelmaxRPM),
                                                m_robotContainer.m_flywheel)
                                .andThen(new WaitCommand(0.25)).andThen(new InstantCommand(
                                                () -> m_robotContainer.m_indexer.runIn(),
                                                m_robotContainer.m_indexer))
                                .andThen(new WaitCommand(0.25)));

                NamedCommands.registerCommand("aim off", new InstantCommand(
                                () -> PPHolonomicDriveController
                                                .setRotationTargetOverride(null)));
                NamedCommands.registerCommand("aim on", new InstantCommand(
                                () -> PPHolonomicDriveController
                                                .setRotationTargetOverride(() -> getRotationTargetOverride())));
        }

        public Command getAutoChooser() {
                return autoChooser.getSelected();
        }
}