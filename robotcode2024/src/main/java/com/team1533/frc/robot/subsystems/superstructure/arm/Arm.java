package com.team1533.frc.robot.subsystems.superstructure.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

    private ProfiledPIDController pivotController;

    @AutoLogOutput
    public BooleanSupplier disableSupplier;

    @AutoLogOutput
    public static double shootingAngle = 0;

    @AutoLogOutput
    public static double shuttlingAngle = 0;

    @AutoLogOutput
    private boolean shooterPIDEnabled;

    @AutoLogOutput
    private double pivotSetpoint;

    @AutoLogOutput
    private double pivotPower;

    public InterpolatingDoubleTreeMap shootingData = new InterpolatingDoubleTreeMap();
    public Translation3d speakerTranslation3d = new Translation3d(0, 5.6282082, 2 + 0.035);
    private static SwerveDrive m_swerve;

    private static Arm instance;

    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public static Arm getInstance() {
        return instance;
    }

    public static Arm initialize(ArmIO io) {
        if (instance == null) {
            instance = new Arm(io, m_swerve);
        }
        return instance;
    }

    @RequiredArgsConstructor
    public enum Goal {
        STOP(() -> 0),
        AIM(() -> shootingAngle),
        STOW(() -> 0),
        MANUAL(() -> 0),
        SUBWOOFER(() -> -5),
        PODIUM(() -> -22),
        PREPARE_CLIMB(() -> -15),
        CLIMB(() -> 0),
        UNTRAP(() -> 0),
        SHUTTLE(() -> shuttlingAngle),
        CUSTOM(() -> -20);

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

    public Arm(ArmIO armIO, SwerveDrive m_swerve) {
        super();

        this.m_swerve = m_swerve;

        disableSupplier = DriverStation::isDisabled;

        pivotController = new ProfiledPIDController(ArmConstants.pivotkP,
                ArmConstants.pivotkI, ArmConstants.pivotkD,
                new Constraints(ArmConstants.kMaxAngularPivotSpeedDegreesPerSecond,
                        ArmConstants.kMaxAngularPivotAccelerationDegreesPerSecondSquared));

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

    public void moveShooter(double motorPivotPower) {

        if (Math.abs(motorPivotPower) < 0.05) {
            shooterPIDEnabled = true;
        } else {
            pivotPower = motorPivotPower;
            io.runPower(pivotPower);
            pivotSetpoint = inputs.posDeg;
            pivotController.reset(pivotSetpoint);
            shooterPIDEnabled = false;
        }
    }

    public void setShooterAngle(double angle) {
        pivotSetpoint = angle;
        shooterPIDEnabled = true;
    }

    public double getAngle() {
        return inputs.posDeg;
    }

    public void resetPIDs() {
        pivotSetpoint = inputs.posDeg;
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
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        setAutoShootAngleDeg();

        if (goal != Goal.MANUAL) {
            pivotSetpoint = goal.getDeg();
        }
        if (disableSupplier.getAsBoolean() || goal == goal.STOP) {
            io.stop();
        }
        if (shooterPIDEnabled) {

            pivotPower = pivotController.calculate(inputs.posDeg, pivotSetpoint);
        }
        if (!inputs.absoluteEncoderConnected) {
            pivotPower = 0;
        }

        io.runPower(pivotPower);
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("cannon degree", inputs.posDeg);
        SmartDashboard.putBoolean("Is Encoder Plugged", inputs.absoluteEncoderConnected);
        SmartDashboard.putNumber("cannon angle setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Cannon Power", pivotPower);
    }

}