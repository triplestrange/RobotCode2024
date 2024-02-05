package frc.robot.subsystems.cannon;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private CANSparkMax lFlyWheel;
    private CANSparkMax rFlyWheel;

    private SparkPIDController lFWController;
    private SparkPIDController rFWController;

    private RelativeEncoder lFWEncoder;
    private RelativeEncoder rFWEncoder;

    private CANSparkMax lPivot;
    private CANSparkMax rPivot;

    private SparkPIDController lPController;
    private SparkPIDController rPController;

    private RelativeEncoder lPEncoder;
    private RelativeEncoder rPEncoder;

    private boolean shooterPIDEnabled;
    
    private double shooterAngle;
    private double flyWheelSpeed;

    private double pivotSetpoint;
    
    private double lFlyWheelSetpoint;
    private double rFlyWheelSetpoint;

    private DutyCycleEncoder pivotAbsoluteEncoder;


    Shooter()   {
        super();

        lFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELL, MotorType.kBrushless);
        rFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELR, MotorType.kBrushless);
        lPivot = new CANSparkMax(Constants.CAN.PIVOTL, MotorType.kBrushless);
        rPivot = new CANSparkMax(Constants.CAN.PIVOTR, MotorType.kBrushless);

        lFlyWheel.restoreFactoryDefaults();
        rFlyWheel.restoreFactoryDefaults();
        lPivot.restoreFactoryDefaults();
        rPivot.restoreFactoryDefaults();

        lFWController = lFlyWheel.getPIDController();
        lFWEncoder = lFlyWheel.getEncoder();

        rFWController = rFlyWheel.getPIDController();
        rFWEncoder = rFlyWheel.getEncoder();

        lPController = lPivot.getPIDController();
        lPEncoder = lPivot.getEncoder();

        rPController = rPivot.getPIDController();
        rPEncoder = rPivot.getEncoder();

        pivotAbsoluteEncoder = new DutyCycleEncoder(Constants.ELECTRICAL.pivotAbsInput);
        pivotAbsoluteEncoder.setPositionOffset(Constants.ShooterConstants.pivotAbsOffset);
        pivotAbsoluteEncoder.setDistancePerRotation(Constants.ShooterConstants.pivotAbsConv);

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
        lFWController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.flyWheelallowedErr, smartMotionSlot);
        
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
        rFWController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.flyWheelallowedErr, smartMotionSlot);
        
        // pivot pid
        lPController.setP(Constants.ShooterConstants.pivotkP);
        lPController.setI(Constants.ShooterConstants.pivotkI);
        lPController.setD(Constants.ShooterConstants.pivotkD);
        lPController.setOutputRange(Constants.ShooterConstants.pivotkMinOutput, 
        Constants.ShooterConstants.pivotkMaxOutput);

        lPController.setSmartMotionMaxVelocity(Constants.ShooterConstants.pivotmaxVel, smartMotionSlot);
        lPController.setSmartMotionMinOutputVelocity(Constants.ShooterConstants.pivotminVel, smartMotionSlot);
        lPController.setSmartMotionMaxAccel(Constants.ShooterConstants.pivotmaxAcc, smartMotionSlot);
        lPController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.pivotallowedErr, smartMotionSlot);

        rPController.setP(Constants.ShooterConstants.pivotkP);
        rPController.setI(Constants.ShooterConstants.pivotkI);
        rPController.setD(Constants.ShooterConstants.pivotkD);
        rPController.setOutputRange(Constants.ShooterConstants.pivotkMinOutput, 
        Constants.ShooterConstants.pivotkMaxOutput);

        rPController.setSmartMotionMaxVelocity(Constants.ShooterConstants.pivotmaxVel, smartMotionSlot);
        rPController.setSmartMotionMinOutputVelocity(Constants.ShooterConstants.pivotminVel, smartMotionSlot);
        rPController.setSmartMotionMaxAccel(Constants.ShooterConstants.pivotmaxAcc, smartMotionSlot);
        rPController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.pivotallowedErr, smartMotionSlot);
    }

    public double getAngle()    {
        return pivotAbsoluteEncoder.getAbsolutePosition();
    }

    @Override
    public void periodic() {
    if(shooterPIDEnabled) {
      lPController.setReference(pivotSetpoint, CANSparkMax.ControlType.kSmartMotion);
    }    

}
    
    public void updateSmartDashBoard()  {

    }
    

}
