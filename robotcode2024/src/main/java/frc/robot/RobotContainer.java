// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickButtons;
import frc.robot.commands.AutoMain;
import frc.robot.subsystems.cannon.Conveyor;
import frc.robot.subsystems.cannon.FlyWheel;
import frc.robot.subsystems.cannon.Rails;
import frc.robot.subsystems.cannon.Shooter;
import frc.robot.subsystems.intake.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.commands.DefaultDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final Robot m_robot;
        public final SwerveDrive m_robotDrive;
        public final Elevator m_elevator;
        public final Intake m_intake;
        public final Shooter m_shooter;
        public final FlyWheel m_flywheel;
        public final Rails m_rails;
        public final Conveyor m_conveyor;

        // private final SendableChooser<Command> choose;
        // public final AutoMain m_Autos;

        // The driver's controller
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(Robot m_Robot) {
                this.m_robot = m_Robot;
                m_robotDrive = new SwerveDrive(m_Robot);
                m_elevator = new Elevator();
                m_intake = new Intake();
                m_shooter = new Shooter();
                m_rails = new Rails();
                m_conveyor = new Conveyor();
                m_flywheel = new FlyWheel();

                configureButtonBindings();

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
                m_robotDrive.setDefaultCommand(

                                // swerve code
                                // The left stick controls tran slation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new DefaultDrive(m_robotDrive, 4.7, 2));// 2.5, 1));

                JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
                JoystickButtons.drWing.onTrue(new InstantCommand(m_robotDrive::setXWheels, m_robotDrive));

         /*        new Trigger(() -> Math.abs(JoystickButtons.m_driverController.getLeftTriggerAxis()) > 0.05)
                                .onTrue(new InstantCommand(() -> {
                                        m_robotDrive.setPresetEnabled(true, -180.0);
                                }));

                new Trigger(() -> Math.abs(JoystickButtons.m_driverController.getRightTriggerAxis()) > 0.05)
                                .onTrue(new InstantCommand(() -> {
                                        m_robotDrive.setPresetEnabled(true, 0);

                                }));
*/
                JoystickButtons.dDpadL.onTrue(new InstantCommand(() -> m_robotDrive.setPresetEnabled(true, 90)));
                JoystickButtons.dDpadR.onTrue(new InstantCommand(() -> m_robotDrive.setPresetEnabled(true, -90)));

                // m_elevator.setDefaultCommand(new RunCommand(
                //                 () -> m_elevator.moveElev(
                //                                 0 * JoystickButtons.m_operatorController.getLeftY(),
                //                                 0 * JoystickButtons.m_operatorController.getRightY()),
                //                 m_elevator));
                
                JoystickButtons.opY.onTrue(new InstantCommand(() -> m_elevator.setIntakePosition(Constants.MechPositions.stowIntakePos), m_elevator));
                JoystickButtons.opA.onTrue(new InstantCommand(() -> m_elevator.setIntakePosition(Constants.MechPositions.groundIntakePos), m_elevator));
                //shooter controls
                m_shooter.setDefaultCommand(new RunCommand(
                                () -> m_shooter.moveShooter(
                                                0.2 * (JoystickButtons.m_driverController.getRightTriggerAxis()-
                                                JoystickButtons.m_driverController.getLeftTriggerAxis())),
                                m_shooter));

                JoystickButtons.drBump.whileTrue(new RunCommand(() -> m_flywheel.setFWSpeed(-5767), m_flywheel));
                JoystickButtons.drBump.onFalse(new InstantCommand(() -> m_flywheel.flyWheelOff(), m_flywheel));
                // m_rails.setDefaultCommand(new RunCommand(
                // () -> m_rails.moveClimb(
                // -JoystickButtons.m_operatorController.getRightY(),
                // -JoystickButtons.m_operatorController.getLeftY()),
                // m_rails));

                // Conveyor
                JoystickButtons.oprBump.whileTrue(new RunCommand(() -> m_intake.runIntake(), m_intake));
                JoystickButtons.oplBump.whileTrue(new RunCommand(() -> m_intake.runOutake(),m_intake));
                m_intake.setDefaultCommand(new RunCommand(() -> m_intake.intakeOff(), m_intake));
                 JoystickButtons.oprBump.whileTrue(new RunCommand(() -> m_conveyor.runConvOut(), m_conveyor));
                m_conveyor.setDefaultCommand(new RunCommand(() -> m_conveyor.conveyorOff(), m_conveyor));
        }

        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

}