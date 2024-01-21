package frc.robot.commands;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoMain extends Command {

        SwerveDrive m_Drive;

        // Commands for AutoRoutines
        public static HashMap<String, Command> eventMap;

        public void eventMapEvents(SwerveDrive m_Drive) {
        };

        public AutoMain(SwerveDrive m_Drive) {
                // Class Variables
                this.m_Drive = m_Drive;
        }

        // Base Commands
      
        
}