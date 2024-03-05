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

    private CANSparkMax lPivot;
    private CANSparkMax rPivot;

    private ProfiledPIDController pivotController;

    private DutyCycleEncoder pivotEncoder;

    private boolean shooterPIDEnabled;

    private double pivotSetpoint;

    private double pivotPower;

    /**
     * Creates a new Shooter.
     */

    public Shooter() {
        super();

        lPivot = new CANSparkMax(Constants.CAN.PIVOTL, MotorType.kBrushless);
        rPivot = new CANSparkMax(Constants.CAN.PIVOTR, MotorType.kBrushless);

        lPivot.restoreFactoryDefaults();
        rPivot.restoreFactoryDefaults();

        lPivot.setSmartCurrentLimit(Constants.ELECTRICAL.shooterPivotCurrentLimit);
        rPivot.setSmartCurrentLimit(Constants.ELECTRICAL.shooterPivotCurrentLimit);

        pivotController = new ProfiledPIDController(Constants.ShooterConstants.pivotkP,
                Constants.ShooterConstants.pivotkI, Constants.ShooterConstants.pivotkD,
                new Constraints(Constants.ShooterConstants.kMaxAngularPivotSpeedDegreesPerSecond,
                        Constants.ShooterConstants.kMaxAngularPivotAccelerationDegreesPerSecondSquared));
        pivotEncoder = new DutyCycleEncoder(Constants.ELECTRICAL.pivotAbsInput);

        pivotEncoder.setPositionOffset(Constants.ShooterConstants.pivotAbsOffset);

        lPivot.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        rPivot.follow(lPivot, false);

        int smartMotionSlot = 0;
    }

    public double getAngle() {

        return MathUtil.inputModulus(
                -pivotEncoder.getAbsolutePosition() * 360 - Constants.ShooterConstants.pivotAbsOffset, 30, -330);

    }

    public void moveShooter(double motorPivotPower) {
        if ((getAngle() >= Constants.ShooterConstants.maxAngle - Constants.ShooterConstants.safeZone)
                && motorPivotPower > 0) {
            motorPivotPower = 0;
        }
        if ((getAngle() <= Constants.ShooterConstants.minAngle + Constants.ShooterConstants.safeZone)
                && motorPivotPower < 0) {
            motorPivotPower = 0;
        }

        if (Math.abs(motorPivotPower) < 0.05) {
            shooterPIDEnabled = true;
        } else {
            pivotPower = motorPivotPower;
            lPivot.set(pivotPower);
            pivotSetpoint = getAngle();
            pivotController.reset(pivotSetpoint);
            shooterPIDEnabled = false;
        }
    }

    public void setShooterAngle(double angle) {
        pivotSetpoint = angle;
        shooterPIDEnabled = true;
    }

    public void resetPIDs() {
        pivotSetpoint = getAngle();
        pivotController.reset(pivotSetpoint);
        shooterPIDEnabled = true;

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

    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("degree", getAngle());
        SmartDashboard.putBoolean("Is Encoder Plugged", pivotEncoder.isConnected());
        SmartDashboard.putNumber("angle setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Power", pivotPower);
    }

}
