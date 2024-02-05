package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax rollers;
    double intakeSpeed = 1;

    /** Creates a new Intake. */
    public Intake() {
        super();
        rollers = new CANSparkMax(Constants.CAN.ROLLERS, MotorType.kBrushless);

        rollers.setSmartCurrentLimit(20);
        rollers.setIdleMode(IdleMode.kBrake);
        // encoder to determine offset angle
    }

    public void runIntake() {
        rollers.set(-intakeSpeed);
    }

    public void runOutake() {
        rollers.set(intakeSpeed);
    }

    public void intakeOff() {
        rollers.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}