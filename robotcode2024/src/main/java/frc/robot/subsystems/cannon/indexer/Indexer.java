package frc.robot.subsystems.cannon.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final CANSparkMax indexer;

    private final DigitalInput indexerInput;

    /** Creates a new Intake. */
    public Indexer() {
        super();
        indexer = new CANSparkMax(Constants.CAN.indexer, MotorType.kBrushless);
        indexer.setSmartCurrentLimit(Constants.ELECTRICAL.indexerCurrentLimit);
        indexer.setIdleMode(IdleMode.kBrake);
        indexerInput = new DigitalInput(Constants.ELECTRICAL.indexerDigitalInput);

        // encoder to determine offset angle
    }

    public void runConvIn() {
        indexer.set(-Constants.IndexerConstants.indexerSpeed);

    }

    public void runConvOut() {
        indexer.set(Constants.IndexerConstants.indexerSpeed);
    }

    public void indexerOff() {
        indexer.set(0);
    }

    public boolean getindexerSensor() {
        return !indexerInput.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("conveor sensor", getindexerSensor());
    }
}