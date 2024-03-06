package frc.robot.subsystems.cannon;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlyWheel extends SubsystemBase {

    private CANSparkMax lFlyWheel;
    private CANSparkMax rFlyWheel;

    private SparkPIDController lFWController;
    private SparkPIDController rFWController;

    private RelativeEncoder lFWEncoder;
    private RelativeEncoder rFWEncoder;

    private double lFlyWheelSetpoint;
    private double rFlyWheelSetpoint;

    /**
     * Creates a new Shooter.
     */

    public FlyWheel() {
        super();

        lFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELL, MotorType.kBrushless);
        rFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELR, MotorType.kBrushless);

        lFlyWheel.restoreFactoryDefaults();
        rFlyWheel.restoreFactoryDefaults();

        lFlyWheel.setSmartCurrentLimit(Constants.ELECTRICAL.flyWheelCurrentLimit);
        rFlyWheel.setSmartCurrentLimit(Constants.ELECTRICAL.flyWheelCurrentLimit);

        lFWController = lFlyWheel.getPIDController();
        lFWEncoder = lFlyWheel.getEncoder();

        rFWController = rFlyWheel.getPIDController();
        rFWEncoder = rFlyWheel.getEncoder();

        int smartMotionSlot = 0;

        // fly wheel
        lFWController.setP(Constants.ShooterConstants.flyWheelkP);
        lFWController.setI(Constants.ShooterConstants.flyWheelkI);
        lFWController.setD(Constants.ShooterConstants.flyWheelkD);
        lFWController.setIZone(Constants.ShooterConstants.flyWheelkIz);
        lFWController.setFF(Constants.ShooterConstants.flyWheelkFF);
        lFWController.setOutputRange(Constants.ShooterConstants.flyWheelkMinOutput,
                Constants.ShooterConstants.flyWheelkMaxOutput);

        lFWController.setSmartMotionMaxVelocity(Constants.ShooterConstants.flyWheelmaxVel, smartMotionSlot);
        lFWController.setSmartMotionMinOutputVelocity(Constants.ShooterConstants.flyWheelminVel, smartMotionSlot);
        lFWController.setSmartMotionMaxAccel(Constants.ShooterConstants.flyWheelmaxAcc, smartMotionSlot);
        lFWController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.flyWheelallowedErr,
                smartMotionSlot);

        rFWController.setP(Constants.ShooterConstants.flyWheelkP);
        rFWController.setI(Constants.ShooterConstants.flyWheelkI);
        rFWController.setD(Constants.ShooterConstants.flyWheelkD);
        rFWController.setIZone(Constants.ShooterConstants.flyWheelkIz);
        rFWController.setFF(Constants.ShooterConstants.flyWheelkFF);
        rFWController.setOutputRange(Constants.ShooterConstants.flyWheelkMinOutput,
                Constants.ShooterConstants.flyWheelkMaxOutput);

        rFWController.setSmartMotionMaxVelocity(Constants.ShooterConstants.flyWheelmaxVel, smartMotionSlot);
        rFWController.setSmartMotionMinOutputVelocity(Constants.ShooterConstants.flyWheelminVel, smartMotionSlot);
        rFWController.setSmartMotionMaxAccel(Constants.ShooterConstants.flyWheelmaxAcc, smartMotionSlot);
        rFWController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.flyWheelallowedErr,
                smartMotionSlot);
    }

    public double getLeftSpeed() {
        return lFWEncoder.getVelocity();
    }

    public double getRightSpeed() {
        return rFWEncoder.getVelocity();
    }

    public void setFWSpeed(double RPM) {
        lFlyWheelSetpoint = RPM;
        rFlyWheelSetpoint = Math.abs(RPM) - Constants.ShooterConstants.rotationalSpeed / 2;

    }

    public void flyWheelOn() {
        lFlyWheelSetpoint = -900;
        rFlyWheelSetpoint = 900;
    }

    public void flyWheelOff() {
        lFlyWheelSetpoint = 0;
        rFlyWheelSetpoint = 0;

    }

    @Override
    public void periodic() {
        lFWController.setReference(lFlyWheelSetpoint, CANSparkMax.ControlType.kVelocity);
        rFWController.setReference(rFlyWheelSetpoint, CANSparkMax.ControlType.kVelocity);

    }

    public void updateSmartDashBoard() {

        SmartDashboard.putNumber("rpm", lFlyWheel.getEncoder().getVelocity());

    }

}
