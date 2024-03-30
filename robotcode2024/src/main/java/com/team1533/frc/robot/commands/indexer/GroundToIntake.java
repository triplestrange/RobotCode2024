package com.team1533.frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;

import com.team1533.frc.robot.subsystems.rollers.Intake;

public class GroundToIntake extends Command {
    private Intake m_Intake;

    /**
     * Creates a new Drive.
     * 
     * normal = 2.5
     * slow = 0.75
     */
    public GroundToIntake(Intake m_Intake) {
        addRequirements(m_Intake);
        this.m_Intake = m_Intake;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Intake.runIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Intake.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Intake.getIntakeSensor();
    }
}
