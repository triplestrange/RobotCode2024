package frc.robot.commands;

import java.time.Instant;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.conveyor.GroundToConveyor;
import frc.robot.commands.conveyor.IntakeToConveyor;
import frc.robot.subsystems.intake.Elevator.IntakePosition;

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
                // Some condition that should decide if we want to override rotation
                if (m_robotContainer.m_conveyor.getConveyorSensor()) {
                        // Return an optional containing the rotation override (this should be a field
                        // relative rotation)
                        return Optional.of(m_robotContainer.m_shoot.rotationToSpeaker());
                } else {
                        // return an empty optional when we don't want to override the path's rotation
                        return Optional.empty();
                }
        }

        public void registerCommands() {
                // Elev Commands
                NamedCommands.registerCommand("amp", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.ampIntakePos), m_robotContainer.m_elevator));
                NamedCommands.registerCommand("ground", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.groundIntakePos)));

                NamedCommands.registerCommand("stowElev", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.stowIntakePos),
                                m_robotContainer.m_elevator));
                // Intake Commands
                NamedCommands.registerCommand("intakeOut", new InstantCommand(() -> m_robotContainer.m_intake
                                .runOutake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeIn", new InstantCommand(() -> m_robotContainer.m_intake
                                .runIntake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> m_robotContainer.m_intake
                                .intakeOff(), m_robotContainer.m_intake));

                // Conveyor Commands
                NamedCommands.registerCommand("indexerOut", new InstantCommand(
                                () -> m_robotContainer.m_conveyor.runConvOut(), m_robotContainer.m_conveyor));

                NamedCommands.registerCommand("conveyorIn", new IntakeToConveyor(m_robotContainer.m_conveyor));

                NamedCommands.registerCommand("indexerIn", new IntakeToConveyor(m_robotContainer.m_conveyor));

                NamedCommands.registerCommand("indexerShoot", new InstantCommand(
                                () -> m_robotContainer.m_conveyor.runConvIn(), m_robotContainer.m_conveyor));
                NamedCommands.registerCommand("conveyorShoot", new InstantCommand(
                                () -> m_robotContainer.m_conveyor.runConvIn(), m_robotContainer.m_conveyor));

                NamedCommands.registerCommand("indexerOff", new InstantCommand(
                                () -> m_robotContainer.m_conveyor.conveyorOff(), m_robotContainer.m_conveyor));

                NamedCommands.registerCommand("intakeNote",
                                (new ParallelDeadlineGroup(new IntakeToConveyor(m_robotContainer.m_conveyor),
                                                new InstantCommand(() -> m_robotContainer.m_elevator
                                                                .setIntakePosition(
                                                                                Constants.MechPositions.groundIntakePos))
                                                                .alongWith(
                                                                                new InstantCommand(
                                                                                                () -> m_robotContainer.m_intake
                                                                                                                .runIntake(),
                                                                                                m_robotContainer.m_intake))))
                                                .andThen(new InstantCommand(() -> m_robotContainer.m_elevator
                                                                .setIntakePosition(
                                                                                Constants.MechPositions.stowIntakePos),
                                                                m_robotContainer.m_elevator)));

                // Shooter
                NamedCommands.registerCommand("shoot",
                                (new RunCommand(() -> m_robotContainer.m_shoot.autonomous(),
                                                m_robotContainer.m_robotDrive, m_robotContainer.m_conveyor,
                                                m_robotContainer.m_shooter, m_robotContainer.m_flywheel)
                                                .withTimeout(1.5))
                                                .finallyDo(() -> m_robotContainer.m_shoot.driveTo.cancel()));

                NamedCommands.registerCommand("shoot fixed", new InstantCommand(
                                () -> m_robotContainer.m_shooter.setShooterAngle(-10), m_robotContainer.m_shooter)
                                .alongWith(new InstantCommand(() -> m_robotContainer.m_flywheel.setFWSpeed(-5676),
                                                m_robotContainer.m_flywheel))
                                .andThen(new WaitCommand(1.5)).andThen(new InstantCommand(
                                                () -> m_robotContainer.m_conveyor.runConvIn(),
                                                m_robotContainer.m_conveyor))
                                .andThen(new WaitCommand(0.5)));
        }

        public Command getAutoChooser() {
                return autoChooser.getSelected();
        }
}