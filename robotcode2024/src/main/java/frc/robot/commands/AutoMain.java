package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

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

        }

        public Command getAutoChooser() {
                return autoChooser.getSelected();
        }
}