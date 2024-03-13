package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.indexer.IntakeToIndexer;

public class AutoMain extends Command {

        RobotContainer m_robotContainer;

        private final LoggedDashboardChooser<Command> autoChooser;

        public AutoMain(RobotContainer m_robotContainer) {
                // Class Variables
                this.m_robotContainer = m_robotContainer;

                registerCommands();

                autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());

                SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

        }

        public void registerCommands() {
                // Elev Commands
                NamedCommands.registerCommand("amp", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.ampIntakePos), m_robotContainer.m_elevator));
                NamedCommands.registerCommand("ground", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.groundIntakePos)));

                NamedCommands.registerCommand("stowElev", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.ampIntakePos), m_robotContainer.m_elevator));
                // Intake Commands
                NamedCommands.registerCommand("intakeOut", new InstantCommand(() -> m_robotContainer.m_intake
                                .runOutake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeIn", new InstantCommand(() -> m_robotContainer.m_intake
                                .runIntake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> m_robotContainer.m_intake
                                .intakeOff(), m_robotContainer.m_intake));

                // indexer Commands
                NamedCommands.registerCommand("indexerOut", new InstantCommand(
                                () -> m_robotContainer.m_indexer.runConvOut(), m_robotContainer.m_indexer));

                NamedCommands.registerCommand("indexerIn", new IntakeToIndexer(m_robotContainer.m_indexer));

                NamedCommands.registerCommand("indexerShoot", new InstantCommand(
                                () -> m_robotContainer.m_indexer.runConvIn(), m_robotContainer.m_indexer));

                NamedCommands.registerCommand("indexerOff", new InstantCommand(
                                () -> m_robotContainer.m_indexer.indexerOff(), m_robotContainer.m_indexer));

                // Shooter
                NamedCommands.registerCommand("shoot",
                                (new RunCommand(() -> m_robotContainer.m_shoot.autoShoot(),
                                                m_robotContainer.m_robotDrive, m_robotContainer.m_indexer,
                                                m_robotContainer.m_shooter, m_robotContainer.m_flywheel)
                                                .until(() -> !m_robotContainer.m_shoot.hasNote)
                                                .andThen(new WaitCommand(0.5))
                                                .finallyDo(() -> m_robotContainer.m_shoot.driveTo.cancel())));

                NamedCommands.registerCommand("shoot fixed", new InstantCommand(
                                () -> m_robotContainer.m_shooter.setShooterAngle(0), m_robotContainer.m_shooter)
                                .alongWith(new InstantCommand(() -> m_robotContainer.m_flywheel.setFWSpeed(-5676),
                                                m_robotContainer.m_flywheel)));
        }

        public Command getAutoChooser() {
                return autoChooser.get();
        }
}