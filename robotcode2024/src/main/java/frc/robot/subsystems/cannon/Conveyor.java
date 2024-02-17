package frc.robot.subsystems.cannon;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

    private final CANSparkMax conveyor;

    private final DigitalInput conveyorInput;

    /** Creates a new Intake. */
    public Conveyor() {
        super();
        conveyor = new CANSparkMax(Constants.CAN.CONVEYOR, MotorType.kBrushless);
        conveyor.setSmartCurrentLimit(Constants.ELECTRICAL.conveyorCurrentLimit);
        conveyor.setIdleMode(IdleMode.kBrake);
        conveyorInput = new DigitalInput(Constants.ELECTRICAL.conveyorDigitalInput);

        // encoder to determine offset angle
    }

    public void runConvIn() {
        conveyor.set(-Constants.IntakeConstants.conveyorSpeed);

    }

    public void runConvOut() {
        conveyor.set(Constants.IntakeConstants.conveyorSpeed);
    }

    public void conveyorOff() {
        conveyor.set(0);
    }

    public boolean getConveyorSensor() {
        return !conveyorInput.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}