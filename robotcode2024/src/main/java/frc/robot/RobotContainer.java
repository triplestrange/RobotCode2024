// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickButtons;
import frc.robot.commands.AutoMain;
import frc.robot.subsystems.swerve.SwerveDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final Robot m_Robot;
        public final SwerveDrive m_robotDrive;
  
        // private final SendableChooser<Command> choose;
        // public final AutoMain m_Autos;

        // The driver's controller
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(Robot m_Robot) {
                this.m_Robot = m_Robot;
                m_robotDrive = new SwerveDrive(m_Robot);
             
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {

                // Driver Controls
                JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
                JoystickButtons.drWing.onTrue(new InstantCommand(m_robotDrive::setXWheels, m_robotDrive));

                new Trigger(() -> Math.abs(JoystickButtons.m_driverController.getLeftTriggerAxis()) > 0.05)
                                .onTrue(new InstantCommand(() -> {
                                        m_robotDrive.setPresetEnabled(true, -180.0);
                                }));

                new Trigger(() -> Math.abs(JoystickButtons.m_driverController.getRightTriggerAxis()) > 0.05)
                                .onTrue(new InstantCommand(() -> {
                                        m_robotDrive.setPresetEnabled(true, 0);

                                }));

                JoystickButtons.dDpadL.onTrue(new InstantCommand(() -> m_robotDrive.setPresetEnabled(true, 90)));
                JoystickButtons.dDpadR.onTrue(new InstantCommand(() -> m_robotDrive.setPresetEnabled(true, -90)));

                // Y | high: 29.25, 29.11, 34
                // X | mid: -
                // dR| cube: -42.7,-133.7,43.5
                // dU| cone upright: -34.69, -139.75, 37.55
                // B | feeder slope: -2, -162.5, 115.2
                // dD| cone lying: -56.45, -139.75, 95.84
                // A | default: -0.6, -169.63, 137.6
                // dL| feed slide -2.78, -73.08, -57.43
        

                // // JoystickButtons.oplWing.whileTrue(new InstantCommand(() -> {
                // // m_robotDrive.setPresetEnabled(true, 0);

                // }));
        }

        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

}