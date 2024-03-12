package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax rollers;

    private final DigitalInput intakeInput;

    /** Creates a new Intake. */
    public Intake() {
        super();
        rollers = new CANSparkMax(Constants.CAN.ROLLERS, MotorType.kBrushless);
        rollers.setSmartCurrentLimit(Constants.ELECTRICAL.rollerCurrentLimit);
        rollers.setIdleMode(IdleMode.kBrake);
        intakeInput = new DigitalInput(Constants.ELECTRICAL.intakeDigitalInput);

        // encoder to determine offset angle
    }

    public void runIntake() {
        rollers.set(Constants.IntakeConstants.intakeSpeed);
    }

    public void runOutake() {
        rollers.set(-Constants.IntakeConstants.intakeSpeed);
    }

    public void intakeOff() {
        rollers.set(0);
    }

    public boolean getIntakeSensor() {
        return !intakeInput.get();
    }

    public Boolean getBlocked(DigitalInput proxInput) {
        Boolean blocked = !proxInput.get();
        return blocked;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void updateSmartDashBoard()  {
                SmartDashboard.putBoolean("intake sensor", getIntakeSensor());

    }
}