package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Superstructure extends SubsystemBase {

    RobotContainer m_robotcontainer;

    private Goal currentGoal = Goal.STOW;
    private Goal desiredGoal = Goal.STOW;
    private Goal lastGoal = Goal.STOW;

    private final Arm arm;
    private final Climber climber;
    private final BackpackActuator backpackActuator;

    private Timer goalTimer = new Timer();

    public Superstructure(RobotContainer m_robotcontainer) {
        this.m_robotcontainer = m_robotcontainer;

        setDefaultCommand(setGoalCommand(Goal.STOW));
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
        OFF
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

}
