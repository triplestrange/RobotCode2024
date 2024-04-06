package com.team1533.frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team1533.frc.robot.RobotContainer;
import com.team1533.frc.robot.subsystems.superstructure.arm.Arm;
import com.team1533.frc.robot.subsystems.superstructure.climb.Climber;
import com.team1533.frc.robot.subsystems.superstructure.elevator.Elevator;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  private Goal currentGoal = Goal.STOW;
  private Goal desiredGoal = Goal.STOW;
  private Goal lastGoal = Goal.STOW;

  private final Elevator m_Elevator;
  private final Climber m_Climb;
  private final Arm m_Arm;

  private Timer goalTimer = new Timer();

  public Superstructure(Elevator m_Elevator, Climber m_Climb, Arm m_Arm) {
    this.m_Elevator = m_Elevator;
    this.m_Climb = m_Climb;
    this.m_Arm = m_Arm;

    goalTimer.start();
  }

  public enum Goal {
    AUTO_AIM,
    AIM,
    AMP,
    PREPARE_TRAP,
    STOW,
    CLIMB,
    TRAP,
    PREPARE_CLIMB,
    GROUND,
    SOURCE,
    FEEDER,
    SHUTTLE,
    STOP
  }

  @Override
  public void periodic() {
    // if (DriverStation.isDisabled()) {
    // setDefaultCommand(setGoalCommand(Goal.STOW));
    // }

    // Reset timer
    if (currentGoal != lastGoal) {
      goalTimer.reset();
    }
    lastGoal = currentGoal;

    m_Arm.periodic();
    m_Climb.periodic();
    m_Elevator.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  public void setGoal(Goal goal) {
    switch (goal) {
      case AUTO_AIM -> {
        m_Arm.setGoal(Arm.Goal.AIM);
        m_Elevator.setGoal(Elevator.Goal.GROUND);
      }
      case AIM -> {
        m_Arm.setGoal(Arm.Goal.AIM);
        m_Elevator.setGoal(Elevator.Goal.STOW);
      }
      case PREPARE_TRAP -> {
        m_Arm.setGoal(Arm.Goal.PREPARE_TRAP);
        m_Elevator.setGoal(Elevator.Goal.TRAP);
      }
      case TRAP -> {
        m_Arm.setGoal(Arm.Goal.CLIMB);
        m_Elevator.setGoal(Elevator.Goal.TRAP);

      }
      case AMP -> {
        m_Arm.setGoal(Arm.Goal.PREPARE_CLIMB);
        m_Elevator.setGoal(Elevator.Goal.AMP);
      }
      case STOW -> {
        m_Arm.setGoal(Arm.Goal.STOW);
        m_Elevator.setGoal(Elevator.Goal.STOW);
      }
      case CLIMB -> {
        m_Arm.setGoal(Arm.Goal.CLIMB);
        m_Elevator.setGoal(Elevator.Goal.STOW);
      }
      case PREPARE_CLIMB -> {
        m_Arm.setGoal(Arm.Goal.PREPARE_CLIMB);
        m_Elevator.setGoal(Elevator.Goal.STOW);
      }
      case GROUND -> {
        m_Arm.setGoal(Arm.Goal.AIM);
        m_Elevator.setGoal(Elevator.Goal.GROUND);
      }
      case SOURCE -> {
        m_Arm.setGoal(Arm.Goal.STOW);
        m_Elevator.setGoal(Elevator.Goal.GROUND);
      }
      case FEEDER -> {
        m_Arm.setGoal(Arm.Goal.STOW);
        m_Elevator.setGoal(Elevator.Goal.FEEDER);
      }
      case SHUTTLE -> {
        m_Arm.setGoal(Arm.Goal.SHUTTLE);
        m_Elevator.setGoal(Elevator.Goal.STOW);
      }
      case STOP -> {
        m_Arm.setGoal(Arm.Goal.STOP);
        m_Elevator.setGoal(Elevator.Goal.STOP);
      }

    }

    if (desiredGoal == goal)
      return;
    desiredGoal = goal;
  }

  /** Command to set goal of superstructure */
  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(
        Goal.STOW))
        .withName("Superstructure " + goal);
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && m_Arm.atGoal() && m_Elevator.atGoal();
  }

  @AutoLogOutput(key = "Superstructure/AtArmGoal")
  public boolean atArmGoal() {
    return currentGoal == desiredGoal && m_Arm.atGoal();
  }

  @AutoLogOutput(key = "Superstructure/AtElevatorGoal")
  public boolean atElevatorGoal() {
    return currentGoal == desiredGoal && m_Elevator.atGoal();
  }

}
