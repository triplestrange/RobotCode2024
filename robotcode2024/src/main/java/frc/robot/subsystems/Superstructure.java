package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManager extends SubsystemBase {

    RobotContainer m_robotcontainer;

    public StateManager(RobotContainer m_robotcontainer) {
        this.m_robotcontainer = m_robotcontainer;
    }

    @Override
    public void periodic() {

    }

    public enum state {
        SHOOT,
        AMP,
        TRAP,
        STOW,
        TEST,
        CLIMB,
        GROUND,
        SOURCE,
    }
}
