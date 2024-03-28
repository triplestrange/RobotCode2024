package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.cannon.climb.Climb;
import frc.robot.subsystems.cannon.shooter.Shooter;
import frc.robot.subsystems.intake.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Superstructure extends SubsystemBase {

  private GoalState currentGoal = GoalState.STOW;
  private GoalState desiredGoal = GoalState.STOW;
  private GoalState lastGoal = GoalState.STOW;

  private final Elevator m_Elevator;
  private final Climb m_Climb;
  private final Shooter m_Shooter;

  private Timer goalTimer = new Timer();

  public Superstructure(Elevator m_Elevator, Climb m_Climb, Shooter m_Shooter) {
    this.m_Elevator = m_Elevator;
    this.m_Climb = m_Climb;
    this.m_Shooter = m_Shooter;

    setDefaultCommand(setGoalCommand(GoalState.STOW));
    goalTimer.start();
  }

  public enum GoalState {
    AIM,
    AMP,
    TRAP,
    STOW,
    CLIMB,
    GROUND,
    SOURCE,
    SHUTTLE
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(GoalState.STOW));
    }

    // Reset timer
    if (currentGoal != lastGoal) {
      goalTimer.reset();
    }
    lastGoal = currentGoal;

    switch (currentGoal) {

    }

    m_Shooter.periodic();
    m_Climb.periodic();
    m_Elevator.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  private void setGoal(GoalState goal) {
    if (desiredGoal == goal)
      return;
    desiredGoal = goal;
  }

  /** Command to set goal of superstructure */
  public Command setGoalCommand(GoalState goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(GoalState.STOW))
        .withName("Superstructure " + goal);
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && arm.atGoal() && m_Climb.atGoal();
  }

  @AutoLogOutput(key = "Superstructure/AtArmGoal")
  public boolean atArmGoal() {
    return currentGoal == desiredGoal && arm.atGoal();
  }

}
