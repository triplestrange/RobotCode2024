package frc.robot.commands;

import java.lang.constant.Constable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Elevator.IntakePosition;

public class AutoMain extends Command {

        RobotContainer m_robotContainer;
        private final SendableChooser<Command> autoChooser;

        public AutoMain(RobotContainer m_robotContainer) {
                // Class Variables
                this.m_robotContainer = m_robotContainer;

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

                registerCommands();

        }

        public void registerCommands() {
                // Elev Commands
                NamedCommands.registerCommand("amp", new RunCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.ampIntakePos), m_robotContainer.m_elevator));
                NamedCommands.registerCommand("stowElev", new RunCommand(() -> m_robotContainer.m_elevator
                                .setIntakePosition(Constants.MechPositions.ampIntakePos), m_robotContainer.m_elevator));
                // Intake Commands
                NamedCommands.registerCommand("intakeOut", new RunCommand(() -> m_robotContainer.m_intake
                                .runOutake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeIn", new RunCommand(() -> m_robotContainer.m_intake
                                .runIntake(), m_robotContainer.m_intake));
                NamedCommands.registerCommand("intakeOff", new RunCommand(() -> m_robotContainer.m_intake
                                .intakeOff(), m_robotContainer.m_intake));
                // Shooter
        }

        public Command getAutoChooser() {
                return autoChooser.getSelected();
        }
}