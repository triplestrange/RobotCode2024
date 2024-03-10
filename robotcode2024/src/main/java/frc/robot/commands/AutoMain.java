package frc.robot.commands;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

        public void registerCommands() {
                // Elev Commands
                NamedCommands.registerCommand("amp", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.ampIntakePos), m_robotContainer.m_elevator));
                NamedCommands.registerCommand("ground", new InstantCommand(() -> m_robotContainer.m_elevator.setIntakePosition(Constants.MechPositions.groundIntakePos)));
                
                NamedCommands.registerCommand("stowElev", new InstantCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.ampIntakePos), m_robotContainer.m_elevator));
                // Intake Commands
                NamedCommands.registerCommand("intakeOut", new InstantCommand(() -> m_robotContainer.m_intake
                                .runOutake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeIn", new InstantCommand(() -> m_robotContainer.m_intake
                                .runIntake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> m_robotContainer.m_intake
                                .intakeOff(), m_robotContainer.m_intake));

                // Conveyor Commands
                NamedCommands.registerCommand("conveyorOut", new InstantCommand(() -> m_robotContainer.m_conveyor.runConvOut()));

                NamedCommands.registerCommand("conveyorIn", new IntakeToConveyor(m_robotContainer.m_conveyor));

                NamedCommands.registerCommand("conveyorOff", new InstantCommand(() -> m_robotContainer.m_conveyor.conveyorOff()));
                
                // Shooter
                NamedCommands.registerCommand("shoot", (new RunCommand(() -> m_robotContainer.m_shoot.autoShoot(), m_robotContainer.m_robotDrive, m_robotContainer.m_conveyor, m_robotContainer.m_shooter, m_robotContainer.m_flywheel).until(() -> !m_robotContainer.m_shoot.hasNote).andThen(new WaitCommand(0.5)).finallyDo(() -> m_robotContainer.m_shoot.driveTo.cancel())));

                NamedCommands.registerCommand("shoot fixed", new InstantCommand(() -> m_robotContainer.m_shooter.setShooterAngle(0)).alongWith(new InstantCommand(() -> m_robotContainer.m_flywheel.setFWSpeed(-5676))));
        }

        public Command getAutoChooser() {
                return autoChooser.getSelected();
        }
}