package frc.robot.subsystems.cannon;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {

    private CANSparkMax lFlyWheel;
    private CANSparkMax rFlyWheel;

    private SparkPIDController lFWController;
    private SparkPIDController rFWController;

    private RelativeEncoder lFWEncoder;
    private RelativeEncoder rFWEncoder;

    private CANSparkMax lPivot;
    private CANSparkMax rPivot;

    private ProfiledPIDController pivotController;

    private DutyCycleEncoder pivotEncoder;

    private boolean shooterPIDEnabled;

    private double pivotSetpoint;

    private double lFlyWheelSetpoint;
    private double rFlyWheelSetpoint;

    private double pivotPower;

    /**
     * Creates a new Shooter.
     */

    public Shooter() {
        super();

        lFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELL, MotorType.kBrushless);
        rFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELR, MotorType.kBrushless);
        lPivot = new CANSparkMax(Constants.CAN.PIVOTL, MotorType.kBrushless);
        rPivot = new CANSparkMax(Constants.CAN.PIVOTR, MotorType.kBrushless);

        lFlyWheel.restoreFactoryDefaults();
        rFlyWheel.restoreFactoryDefaults();
        lPivot.restoreFactoryDefaults();
        rPivot.restoreFactoryDefaults();

        lFlyWheel.setSmartCurrentLimit(Constants.ELECTRICAL.flyWheelCurrentLimit);
        rFlyWheel.setSmartCurrentLimit(Constants.ELECTRICAL.flyWheelCurrentLimit);
        lPivot.setSmartCurrentLimit(Constants.ELECTRICAL.shooterPivotCurrentLimit);
        rPivot.setSmartCurrentLimit(Constants.ELECTRICAL.shooterPivotCurrentLimit);

        lFWController = lFlyWheel.getPIDController();
        lFWEncoder = lFlyWheel.getEncoder();

        rFWController = rFlyWheel.getPIDController();
        rFWEncoder = rFlyWheel.getEncoder();

        pivotController = new ProfiledPIDController(Constants.ShooterConstants.pivotkP,
                Constants.ShooterConstants.pivotkI, Constants.ShooterConstants.pivotkD, new Constraints(0, 0));
        pivotEncoder = new DutyCycleEncoder(3);

        pivotEncoder.setPositionOffset(Constants.ShooterConstants.pivotAbsOffset);

        lPivot.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

        rPivot.follow(lPivot, true);

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

    public double getAngle() {
        return MathUtil
                .angleModulus(2 * Math.PI * pivotEncoder.getAbsolutePosition());
    }

    public void moveShooter(double motorPivotPower) {
        if (getAngle() >= Constants.ShooterConstants.maxAngle - Constants.ShooterConstants.safeZone) {
            motorPivotPower = 0;
        }
        if (getAngle() <= Constants.ShooterConstants.minAngle + Constants.ShooterConstants.safeZone) {
            motorPivotPower = 0;
        }

        shooterPIDEnabled = true;
        if (Math.abs(motorPivotPower) < 0.05) {
        } else {
            pivotPower = motorPivotPower;
            lPivot.set(pivotPower);
            pivotSetpoint = getAngle();
            pivotController.reset(pivotSetpoint);
            shooterPIDEnabled = false;
        }
    }

    public void setShooterPos(double angle) {
        pivotSetpoint = angle;
        shooterPIDEnabled = true;
    }

    public double getLeftSpeed() {
        return lFWEncoder.getVelocity();
    }

    public double getRightSpeed() {
        return rFWEncoder.getVelocity();
    }

    public void setFWSpeed(double RPM) {
        lFlyWheelSetpoint = RPM;
        rFlyWheelSetpoint = -(RPM - Constants.ShooterConstants.rotationalSpeed);

    }

    public void flyWheelOff() {
        lFlyWheelSetpoint = 0;
        rFlyWheelSetpoint = 0;
    }

    @Override
    public void periodic() {
        if (shooterPIDEnabled) {
            pivotPower = pivotController.calculate(getAngle(), pivotSetpoint);
        }
        if (!pivotEncoder.isConnected()) {
            pivotPower = 0;
        }
        lPivot.set(pivotPower);
        lFWController.setReference(lFlyWheelSetpoint, CANSparkMax.ControlType.kVelocity);
        rFWController.setReference(rFlyWheelSetpoint, CANSparkMax.ControlType.kVelocity);

    }

    public void updateSmartDashBoard() {

        SmartDashboard.putNumber("rpm", lFlyWheel.getEncoder().getVelocity());


    }

}
