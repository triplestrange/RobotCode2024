package frc.robot.subsystems.cannon.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    private CANSparkMax lPivot;
    private CANSparkMax rPivot;

    private ProfiledPIDController pivotController;

    private DutyCycleEncoder pivotEncoder;

    private boolean shooterPIDEnabled;

    private double pivotSetpoint;

    private double pivotPower;

    public double shootingAngle = 0;

            public InterpolatingDoubleTreeMap shootingData = new InterpolatingDoubleTreeMap();
                public Translation3d speakerTranslation3d = new Translation3d(0, 5.6282082, 2 + 0.035);
    private SwerveDrive m_swerve;



    /**
     * Creates a new Shooter.
     */

    public Shooter(SwerveDrive m_swerve) {
        super();

        this.m_swerve = m_swerve;

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

        lPivot.setInverted(true);
        pivotController.setIZone(1);
        int smartMotionSlot = 0;

         shootingData.put(1.0, 0.0);
        shootingData.put(1.655738, -12.2);
        shootingData.put(2.2, -16.0);
        shootingData.put(3.120114, -23.5);
        shootingData.put(4.991135, -31.55);
        shootingData.put(5.3, -31.0);
        shootingData.put(6.38, -33.4);
        shootingData.put(7.39, -34.5);

        

    }

    public double getAngle() {

        return MathUtil.inputModulus(
                -pivotEncoder.getAbsolutePosition() * 360 - Constants.ShooterConstants.pivotAbsOffset, 30, -330);

    }

    public void moveShooter(double motorPivotPower) {
        // if ((getAngle() >= Constants.ShooterConstants.maxAngle -
        // Constants.ShooterConstants.safeZone)
        // && motorPivotPower > 0) {
        // motorPivotPower = 0;
        // }
        // if ((getAngle() <= Constants.ShooterConstants.minAngle +
        // Constants.ShooterConstants.safeZone)
        // && motorPivotPower < 0) {
        // motorPivotPower = 0;
        // }

        if (Math.abs(motorPivotPower) < 0.05) {
            shooterPIDEnabled = true;
        } else {
            pivotPower = motorPivotPower;
            lPivot.set(pivotPower);
            rPivot.set(pivotPower);
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

     public boolean isAllianceRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void enablePID(boolean enable)   {
        pivotSetpoint = shootingAngle;
        shooterPIDEnabled = enable;
    }

    public Translation3d flipTranslation3d(Translation3d translation) {
        return new Translation3d(16.54 - translation.getX(), translation.getY(), translation.getZ());

    }

    @Override
    public void periodic() {
          if (isAllianceRed()) {
            shootingAngle = shootingData.get(m_swerve.getPose().getTranslation()
                    .getDistance(flipTranslation3d(speakerTranslation3d).toTranslation2d()));
        } else {
            shootingAngle = shootingData.get(m_swerve.getPose().getTranslation()
                    .getDistance((speakerTranslation3d.toTranslation2d())));
        }
        if (shooterPIDEnabled) {
            pivotPower = pivotController.calculate(getAngle(), shootingAngle);
        }
        if (!pivotEncoder.isConnected()) {
            pivotPower = 0;
        }


        lPivot.set(pivotPower);
        rPivot.set(pivotPower);
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("cannon degree", getAngle());
        SmartDashboard.putBoolean("Is Encoder Plugged", pivotEncoder.isConnected());
        SmartDashboard.putNumber("cannon angle setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Cannon Power", pivotPower);
    }

}
