package com.team1533.frc.robot.subsystems.cannon.indexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private static Indexer instance;

    private IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public static Indexer getInstance() {
        return instance;
    }

    public static Indexer initialize(IndexerIO io) {
        if (instance == null) {
            instance = new Indexer(io);
        }
        return instance;
    }

    /** Creates a new Indexer. */
    public Indexer(IndexerIO indexerIO) {
        super();

        io = indexerIO;
        io.updateInputs(inputs);

        io.setIdleMode(IdleMode.kBrake);
    }

    public void runIn() {
        io.runSpeed(-1);
    }

    public void runOut() {
        io.runSpeed(1);
    }

    public void indexerOff() {
        io.stop();
    }

    public boolean getIndexerSensor() {
        return inputs.sensor;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("Indexer sensor", getIndexerSensor());

    }
}