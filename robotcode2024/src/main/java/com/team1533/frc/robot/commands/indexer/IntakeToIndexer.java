package com.team1533.frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;

import com.team1533.frc.robot.subsystems.cannon.indexer.Indexer;

public class IntakeToIndexer extends Command {
    private Indexer m_indexer;

    /**
     * Creates a new Drive.
     * 
     * normal = 2.5
     * slow = 0.75
     */
    public IntakeToIndexer(Indexer m_indexer) {
        addRequirements(m_indexer);
        this.m_indexer = m_indexer;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_indexer.runIn();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.indexerOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_indexer.getIndexerSensor();
    }
}
