package com.team1533.frc.robot.commands.indexer;

import com.team1533.frc.robot.subsystems.cannon.indexer.Indexer;
import com.team1533.frc.robot.subsystems.leds.Leds;
import com.team1533.frc.robot.subsystems.leds.Leds.LedMode;
import com.team1533.frc.robot.subsystems.rollers.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class GroundToIndexer extends Command {
    private Indexer m_indexer;
    private Intake m_Intake;
    private Leds m_Leds;

    /**
     * Creates a new Drive.
     * 
     * normal = 2.5
     * slow = 0.75
     */
    public GroundToIndexer(Indexer m_Indexer, Intake m_Intake, Leds m_Leds) {
        addRequirements(m_Indexer, m_Intake, m_Leds);
        this.m_indexer = m_Indexer;
        this.m_Intake = m_Intake;
        this.m_Leds = m_Leds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_indexer.runIn();
        m_Intake.runIntake();
        m_Leds.setMode(LedMode.INTAKING);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!m_Intake.getIntakeSensor()) {
            m_Leds.setMode(LedMode.HAS_NOTE);

        }
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
        return !m_indexer.getIndexerSensor();

    }
}
