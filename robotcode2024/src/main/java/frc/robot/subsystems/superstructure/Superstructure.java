package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Superstructure extends SubsystemBase {

  private Goal currentGoal = Goal.STOW;
  private Goal desiredGoal = Goal.STOW;
  private Goal lastGoal = Goal.STOW;

  private final Elevator m_Elevator;
  private final Climb m_Climb;
  private final Arm m_Arm;

  private Timer goalTimer = new Timer();

  public Superstructure(Elevator m_Elevator, Climb m_Climb, Arm m_Arm) {
    this.m_Elevator = m_Elevator;
    this.m_Climb = m_Climb;
    this.m_Arm = m_Arm;

    setDefaultCommand(setGoalCommand(Goal.STOW));
    goalTimer.start();
  }

  public enum Goal {
    AIM,
    AMP,
    TRAP,
    STOW,
    CLIMB,
    PREPARE_CLIMB,
    GROUND,
    SOURCE,
    FEEDER,
    SHUTTLE,
    STOP
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(Goal.STOW));
    }

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

  private void setGoal(Goal goal) {
    switch (currentGoal) {
      case AIM -> {
        m_Arm.setGoal(Arm.Goal.AIM);
        m_Elevator.setGoal(Elevator.Goal.STOW);
      }
      case TRAP -> {
        m_Arm.setGoal(Arm.Goal.CLIMB);
        m_Elevator.setGoal(Elevator.Goal.TRAP);
      }
      case AMP -> {
        m_Arm.setGoal(Arm.Goal.STOW);
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