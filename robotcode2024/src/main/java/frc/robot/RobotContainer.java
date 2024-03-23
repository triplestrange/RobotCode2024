// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickButtons;
import frc.robot.commands.AutoMain;
import frc.robot.subsystems.cannon.Conveyor;
import frc.robot.subsystems.cannon.FlyWheel;
import frc.robot.subsystems.cannon.Climb;
import frc.robot.subsystems.cannon.Shooter;
import frc.robot.subsystems.intake.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.Vision;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.automations.DriveTo;
import frc.robot.commands.automations.Shoot;
import frc.robot.commands.conveyor.GroundToConveyor;
import frc.robot.commands.conveyor.GroundToIntake;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        public final Robot m_robot;
        public final SwerveDrive m_robotDrive;
        public final Elevator m_elevator;
        public final Intake m_intake;
        public final Shooter m_shooter;
        public final FlyWheel m_flywheel;
        public final Climb m_climb;
        public final Conveyor m_conveyor;
        public Vision m_vision;
        public final Shoot m_shoot;

        // private final SendableChooser<Command> choose;
        public final AutoMain m_Autos;

        private double flywheelSetpoint;

        // The driver's controller
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(Robot m_Robot) {
                this.m_robot = m_Robot;
                m_robotDrive = new SwerveDrive(this);
                m_elevator = new Elevator();
                m_intake = new Intake();
                m_shooter = new Shooter();
                m_climb = new Climb();
                m_conveyor = new Conveyor();
                m_flywheel = new FlyWheel();
                m_shoot = new Shoot(this);
                m_vision = new Vision(m_robotDrive, m_shoot, m_elevator);
                m_Autos = new AutoMain(this);

                configureButtonBindings();

                PPHolonomicDriveController.setRotationTargetOverride(m_shoot::getRotationTargetOverride);


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

                // Swerve Controls

                m_robotDrive.setDefaultCommand(
                                new DefaultDrive(m_robotDrive, 4.7, 2));// 2.5, 1));

                JoystickButtons.dlBump.whileTrue(
                                new DefaultDrive(m_robotDrive, 0.85, 1));

                JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
                JoystickButtons.drWing.onTrue(new InstantCommand(m_robotDrive::setXWheels, m_robotDrive));

                JoystickButtons.dDpadL.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? -90 : 90)));
                JoystickButtons.dDpadU.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? 0 : 180)));
                JoystickButtons.dDpadR.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? 90 : -90)));
                JoystickButtons.dDpadD.onTrue(new InstantCommand(
                                () -> m_robotDrive.setPresetEnabled(true, m_robotDrive.isRedAlliance() ? 180 : 0)));

                // Elevator Controls

                JoystickButtons.opA.onTrue(new InstantCommand(
                                () -> m_elevator.setIntakePosition(Constants.MechPositions.stowIntakePos), m_elevator));
                JoystickButtons.opDpadU.onTrue(new InstantCommand(
                                () -> m_elevator.setIntakePosition(Constants.MechPositions.ampIntakePos), m_elevator));
                JoystickButtons.opDpadD.onTrue(new InstantCommand(
                                () -> m_elevator.setIntakePosition(Constants.MechPositions.groundIntakePos)));

                m_elevator.setDefaultCommand(new RunCommand(
                                () -> m_elevator.moveElev(
                                                1 * JoystickButtons.m_operatorController.getRightY(),
                                                0.3 * JoystickButtons.m_operatorController.getRightX()),
                                m_elevator));

                // Climb Controls

                // m_climb.setDefaultCommand(new RunCommand(
                // () -> m_climb.moveClimb(
                // 0.6 * JoystickButtons.m_operatorController.getLeftY(),
                // 0.6 * JoystickButtons.m_operatorController.getLeftY()),
                // m_climb));

                // Pivot Controls

                JoystickButtons.opB.onTrue(new InstantCommand(
                                () -> m_shooter.setShooterAngle(Constants.MechPositions.climbPivotPos)));

                JoystickButtons.opX.whileTrue(new InstantCommand(
                                () -> m_shooter.setShooterAngle(Constants.MechPositions.clearancePivotPos)));
                JoystickButtons.opY.whileTrue(new InstantCommand(
                                () -> m_shooter.setShooterAngle(Constants.MechPositions.lowPivotPos)));

                m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.moveShooter(
                                0.1 * JoystickButtons.m_operatorController.getLeftY()), m_shooter));
                // Intake and Conveyor Controls

                JoystickButtons.oprBump.whileTrue(new RunCommand(() -> m_intake.runIntake(), m_intake)
                                .alongWith(new RunCommand(() -> m_conveyor.runConvIn(), m_conveyor)));

                // JoystickButtons.oprBump.whileTrue(new InstantCommand(() ->
                // m_intake.runIntake()).andThen(new InstantCommand(() ->
                // m_intake.intakeOff())));

                JoystickButtons.opDpadR.whileTrue(new GroundToIntake(m_intake));

                JoystickButtons.opDpadL.whileTrue((new GroundToConveyor(m_conveyor, m_intake))
                                .andThen(new RunCommand(() -> m_conveyor.runConvOut(), m_conveyor)
                                                .until(() -> !m_conveyor.getConveyorSensor())));

                JoystickButtons.oplBump.whileTrue(new RunCommand(() -> m_intake.runOutake(), m_intake)
                                .alongWith(new RunCommand(() -> m_conveyor.runConvOut(), m_conveyor)));

                m_intake.setDefaultCommand(new InstantCommand(() -> m_intake.intakeOff(), m_intake));

                m_conveyor.setDefaultCommand(new InstantCommand(() -> m_conveyor.conveyorOff(), m_conveyor));

                // Fly Wheels controls

                if (m_shoot.isAllianceRed()) {
                        flywheelSetpoint = m_robotDrive.getPose().getTranslation().getDistance(
                                        m_shoot.flipTranslation3d(m_shoot.speakerTranslation3d).toTranslation2d())
                                        * 4850.0 / 3;
                } else {
                        flywheelSetpoint = m_robotDrive.getPose().getTranslation()
                                        .getDistance(m_shoot.speakerTranslation3d.toTranslation2d()) * 4850.0 / 3;
                }

                flywheelSetpoint = MathUtil.clamp(flywheelSetpoint, 2500, 4850);

                JoystickButtons.drBump.whileTrue(new RunCommand(() -> m_flywheel.setFWSpeed(-flywheelSetpoint)));

                m_flywheel.setDefaultCommand(new RunCommand(() -> m_flywheel.flyWheelOff(), m_flywheel));

                // Shooting Automations

                JoystickButtons.dX.whileTrue(
                                new RunCommand(() -> m_shoot.autoShoot(), m_shooter, m_flywheel, m_conveyor))
                                .onFalse(new InstantCommand(() -> m_shoot.driveTo.cancel())
                                                .alongWith(new InstantCommand(() -> m_shooter.setShooterAngle(
                                                                Constants.MechPositions.climbPivotPos))));
                // Amp Automations

                JoystickButtons.dB
                                .whileTrue(new DriveTo(Constants.MechPositions.amp, 0, 0, m_robotDrive, m_robot)
                                                .alongWith(new InstantCommand(() -> m_elevator.setIntakePosition(
                                                                Constants.MechPositions.ampIntakePos))));
        }

        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

}