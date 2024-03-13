package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.cannon.indexer.Indexer;
import frc.robot.subsystems.intake.rollers.Intake;

public class GroundToIndexer extends Command {
    private Indexer m_indexer;
    private Intake m_Intake;

    /**
     * Creates a new Drive.
     * 
     * normal = 2.5
     * slow = 0.75
     */
    public GroundToIndexer(Indexer m_indexer, Intake m_Intake) {
        addRequirements(m_indexer, m_Intake);
        this.m_indexer = m_indexer;
        this.m_Intake = m_Intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_indexer.runConvIn();
        m_Intake.runIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.indexerOff();
        m_Intake.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_indexer.getindexerSensor();
    }
}
