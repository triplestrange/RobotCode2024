// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1533.frc.robot;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.team1533.frc.robot.subsystems.vision.VisionConstants;
import com.team1533.frc.robot.util.Alert;
import com.team1533.frc.robot.util.Alert.AlertType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;
  private int i;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // Set a metadata value

    switch (Constants.LoggerConstants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        try { 
          Logger.addDataReceiver(new WPILOGWriter());
        }

        catch(Exception e) {
           new Alert("The USB Flash Drive is unplugged.", AlertType.WARNING)
            .set(true);
        }

        try {
        Logger.addDataReceiver(new NT4Publisher());

        }

        catch(Exception e)  {
          new Alert("The Logger is not publsihing to Network Tables", AlertType.WARNING).set(true);
        }
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), 0.01));
        break;
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
      String name = command.getName();
      int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
      commandCounts.put(name, count);
      Logger.recordOutput(
          "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
      Logger.recordOutput("CommandsAll/" + name, count > 0);
    };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    RobotController.setBrownoutVoltage(6.0);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(this);
    m_robotContainer.m_robotDrive.zeroHeading();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    i++;
    if (i % 10 == 0) {
      m_robotContainer.m_climb.updateSmartDashBoard();
      m_robotContainer.m_elevator.updateSmartDashBoard();
      m_robotContainer.m_flywheel.updateSmartDashBoard();
      m_robotContainer.m_indexer.updateSmartDashBoard();
      m_robotContainer.m_intake.updateSmartDashBoard();
      m_robotContainer.m_robotDrive.updateSmartDashBoard();
      m_robotContainer.m_Arm.updateSmartDashBoard();
      m_robotContainer.m_vision.updateSmartDashBoard();
      m_robotContainer.m_shoot.updateSmartDashBoard();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

    m_robotContainer.m_Arm.resetPIDs();
    m_robotContainer.m_elevator.resetPIDs();

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_robotDrive.m_odometry
        .setVisionMeasurementStdDevs(VisionConstants.VISION_MEASUREMENT_STD_DEVS);
    m_autonomousCommand = m_robotContainer.m_Autos.getAutoChooser();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_robotContainer.m_robotDrive.m_odometry
    // .setVisionMeasurementStdDevs(Constants.VisionConstants.VISION_MEASUREMENT_STD_DEVS);
    m_robotContainer.m_robotDrive.m_odometry
        .setVisionMeasurementStdDevs(VisionConstants.VISION_MEASUREMENT_STD_DEVS);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}