package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoMain {

        RobotContainer m_robotContainer;        
        private final SendableChooser<Command> autoChooser;

<<<<<<< Updated upstream
        // Commands for AutoRoutines
        public static HashMap<String, Command> eventMap;

        public void eventMapEvents(SwerveDrive m_Drive) {
        };

        public AutoMain(SwerveDrive m_Drive) {
=======
        public AutoMain(RobotContainer m_robotContainer) {
>>>>>>> Stashed changes
                // Class Variables
                this.m_robotContainer = m_robotContainer;

                autoChooser = AutoBuilder.buildAutoChooser();
                
                SmartDashboard.putData("Auto Chooser", autoChooser);

                registerCommands();
                
        }

        public void registerCommands()  {
        }

        public Command getAutoChooser() {
            return autoChooser.getSelected();
        }

        // Base Commands

}