package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    public final CANSparkMax elev;
    public double elevPov;
    public SparkPIDController elevController;
    public RelativeEncoder elevRelativeEncoder;ß


    Elevator()   {
        elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
        elev.setIdleMode(IdleMode.kBrake);
        elev.restoreFactoryDefaults();
 
        elevController = elev.getPIDController();
        elevRelativeEncoder = elev.getEncoder();

    // set PID coefficients
    elevController.setP(Constants.ElevatorConstants.kP);
    elevController.setI(Constants.ElevatorConstants.kI);
    elevController.setD(Constants.ElevatorConstants.kD);
    elevController.setOutputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    elevController.setSmartMotionMaxVelocity(Constants.ElevatorConstants.maxVel, smartMotionSlot);
    elevController.setSmartMotionMinOutputVelocity(Constants.ElevatorConstants.minVel, smartMotionSlot);
    elevController.setSmartMotionMaxAccel(Constants.ElevatorConstants.maxAcc, smartMotionSlot);
    elevController.setSmartMotionAllowedClosedLoopError(Constants.ElevatorConstants.allowedErr, smartMotionSlot);
  }
    public double 

    public void periodic()  {
        elevController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariable = elevRelativeEncoder.getPosition();
    }

    public void updateSmartDashBoard()  {
        
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", Constants.ElevatorConstants.kP);
    SmartDashboard.putNumber("I Gain", Constants.ElevatorConstants.kI);
    SmartDashboard.putNumber("D Gain", Constants.ElevatorConstants.kD);
    SmartDashboard.putNumber("Max Output", Constants.ElevatorConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", Constants.ElevatorConstants.kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", Constants.ElevatorConstants.maxVel);
    SmartDashboard.putNumber("Min Velocity", Constants.ElevatorConstants.minVel);
    SmartDashboard.putNumber("Max Acceleration", Constants.ElevatorConstants.maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", Constants.ElevatorConstants.allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    
    }
}

