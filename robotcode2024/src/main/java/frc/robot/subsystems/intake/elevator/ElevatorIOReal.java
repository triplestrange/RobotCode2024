package frc.robot.subsystems.intake.elevator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
    private CANSparkMax elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
    private CANSparkMax intake = new CANSparkMax(Constants.CAN.IPIVOT, MotorType.kBrushless);
    private RelativeEncoder motorEncoder = pivotLeft.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
    // private AbsoluteEncoder encoder = pivotRight.getAbsoluteEncoder();
    private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
    private double inputVolts = 0.0;

  public PivotIOReal() {


        intake.setInverted(true);
        elev.setInverted(false);

        elev.setIdleMode(IdleMode.kBrake);
        elev.setSmartCurrentLimit(Constants.ELECTRICAL.elevatorCurrentLimit);

        intake.setIdleMode(IdleMode.kBrake);
        intake.setSmartCurrentLimit(Constants.ELECTRICAL.intakeCurrentLimit);

        elevRelativeEncoder = elev.getEncoder();


        elev.burnFlash();
        intake.burnFlash();
  }
}
