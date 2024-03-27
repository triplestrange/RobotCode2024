package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.cannon.climb.Climb;
import frc.robot.subsystems.cannon.shooter.Shooter;
import frc.robot.subsystems.intake.elevator.Elevator;

public class Superstructure extends SubsystemBase {

  private GoalState currentGoal = GoalState.STOW;
  private GoalState desiredGoal = GoalState.STOW;
  private GoalState lastGoal = GoalState.STOW;

  private final Elevator elevator;
  private final Climb climb;
  private final Shooter shooter;

  private Timer goalTimer = new Timer();

  public Superstructure(RobotContainer m_robotcontainer) {
    this.

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
        switch () {
            case value:
                
                break;
        
            default:
                break;
        }
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

  /** Command to aim the superstructure with a compensation value in degrees */
  public Command aimWithCompensation(double compensation) {
    return setGoalCommand(Goal.AIM)
        .beforeStarting(() -> arm.setCurrentCompensation(compensation))
        .finallyDo(() -> arm.setCurrentCompensation(0.0));
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && arm.atGoal() && climber.atGoal();
  }

  @AutoLogOutput(key = "Superstructure/AtArmGoal")
  public boolean atArmGoal() {
    return currentGoal == desiredGoal && arm.atGoal();
  }

}
