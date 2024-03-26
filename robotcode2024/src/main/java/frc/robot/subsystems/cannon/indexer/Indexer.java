package frc.robot.subsystems.cannon.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
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
        io.runVolts(12);
    }

    public void runOut() {
        io.runVolts(-12);
    }

    public void indexerOff() {
        io.stop();
    }

    public boolean getIndexerSensor() {
        return io.getSensor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("Indexer sensor", getIndexerSensor());

    }
}