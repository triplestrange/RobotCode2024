package frc.robot.subsystems.superstructure.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

    private CANSparkMax lPivot;
    private CANSparkMax rPivot;

    private ProfiledPIDController pivotController;

    private DutyCycleEncoder pivotEncoder;

    private boolean shooterPIDEnabled;

    private double pivotSetpoint;

    private double pivotPower;

    public BooleanSupplier disableSupplier;

    public static double shootingAngle = 0;

    public InterpolatingDoubleTreeMap shootingData = new InterpolatingDoubleTreeMap();
    public Translation3d speakerTranslation3d = new Translation3d(0, 5.6282082, 2 + 0.035);
    private SwerveDrive m_swerve;

    @RequiredArgsConstructor
    public enum Goal {
        STOP(() -> 0),
        AIM(() -> shootingAngle),
        STOW(() -> 0),
        MANUAL(() -> 0),
        SUBWOOFER(() -> 0),
        PODIUM(() -> 0),
        PREPARE_CLIMB(() -> 0),
        CLIMB(() -> 0),
        UNTRAP(() -> 0),
        SHUTTLE(() -> 0),
        CUSTOM(() -> 20);

        private final DoubleSupplier armSetpointSupplier;

        private double getDeg() {
            return armSetpointSupplier.getAsDouble();
        }
    }

    @AutoLogOutput
    @Getter
    @Setter
    private Goal goal = Goal.STOW;

    /**
     * Creates a new Shooter.
     */

    public Arm(SwerveDrive m_swerve) {
        super();

        this.m_swerve = m_swerve;

        disableSupplier = DriverStation::isDisabled;

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

        pivotController.setTolerance(0.3);

        shootingData.put(1.0, 0.0);
        shootingData.put(1.5, -3.2);
        shootingData.put(2.0, -9.5);
        shootingData.put(2.5, -15.5);
        shootingData.put(3.0, -19.7);
        shootingData.put(3.5, -23.85);
        shootingData.put(4.0, -25.6);

        // shootingData.put(1.655738, -12.2);
        // shootingData.put(2.2, -16.0);
        // shootingData.put(3.120114, -23.5);
        shootingData.put(4.9, -30.5);
        // // shootingData.put(4.991135, -29.55);
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

    public void setAutoShootAngleDeg() {
        if (isAllianceRed()) {
            shootingAngle = shootingData.get(m_swerve.getPose().getTranslation()
                    .getDistance(flipTranslation3d(speakerTranslation3d).toTranslation2d()));
        } else {
            shootingAngle = shootingData.get(m_swerve.getPose().getTranslation()
                    .getDistance((speakerTranslation3d.toTranslation2d())));

        }
    }

    public void enableAutoShooting(boolean enable) {
        pivotSetpoint = shootingAngle;
        shooterPIDEnabled = enable;
    }

    public Translation3d flipTranslation3d(Translation3d translation) {
        return new Translation3d(16.54 - translation.getX(), translation.getY(), translation.getZ());

    }

    @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
    public boolean atGoal() {
        return pivotController.atGoal();
    }

    public void periodic() {
        setAutoShootAngleDeg();

        if (goal != Goal.MANUAL) {
            pivotSetpoint = goal.getDeg();
        }
        if (disableSupplier.getAsBoolean() || goal == goal.STOP) {
            io.stop();
        }
        if (shooterPIDEnabled) {

            pivotPower = pivotController.calculate(getAngle(), pivotSetpoint);

            // change between pivotSetpoint to shootingAngle for manual & autoshoot
            // respectively
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