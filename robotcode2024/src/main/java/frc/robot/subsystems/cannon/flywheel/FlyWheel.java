package frc.robot.subsystems.cannon.flywheel;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlyWheel extends SubsystemBase {
    private static FlyWheel instance;

    private FlyWheelIO io;
    private FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();

    private double RPM;

    public static FlyWheel getInstance() {
        return instance;
    }

    public static FlyWheel initialize(FlyWheelIO io) {
        if (instance == null) {
            instance = new FlyWheel(io);
        }
        return instance;
    }

    /**
     * Creates a new Shooter.
     */

    public FlyWheel(FlyWheelIO flyWheelIO) {
        super();
        io = flyWheelIO;
        io.updateInputs(inputs);

        io.setIdleMode(IdleMode.kCoast);
    }

    public double getLeftSpeed() {
        return inputs.leftVel;
    }

    public double getRightSpeed() {
        return inputs.rightVel;
    }

    public void setFWSpeed(double RPM) {
        this.RPM = RPM;
    }

    public void flyWheelOn() {
        io.setLeftSpeed(-900);
        io.setRightSpeed(900);
    }

    public void flyWheelOff() {
        io.setLeftSpeed(0);
        io.setRightSpeed(0);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.runSpeed(RPM);

    }

    public void updateSmartDashBoard() {

        SmartDashboard.putNumber("left rpm", inputs.leftVel);
        SmartDashboard.putNumber("right rpm", inputs.rightVel);
        SmartDashboard.putNumber("left setpoint", inputs.leftSetpoint);
        SmartDashboard.putNumber("right setpoint", inputs.rightSetpoint);

    }

}
