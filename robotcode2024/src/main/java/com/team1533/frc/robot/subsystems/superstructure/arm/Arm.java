package com.team1533.frc.robot.subsystems.superstructure.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team1533.frc.robot.commands.automations.Shoot;
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

    public BooleanSupplier disableSupplier;

    @AutoLogOutput
    public static double shootingAngle = 0;
    @AutoLogOutput
    public static double shootingAngleNoComp = 0;

    @AutoLogOutput
    private boolean shooterPIDEnabled = true;

    @AutoLogOutput
    private double pivotSetpoint = 0.0;

    @AutoLogOutput
    private double pivotPower = 0.0;

    public InterpolatingDoubleTreeMap shootingData = new InterpolatingDoubleTreeMap();
    public Translation3d speakerTranslation3d = new Translation3d(0, 5.6282082, 2 + 0.035);
    private static SwerveDrive m_swerve;
    private static Shoot m_shoot;

    private static Arm instance;

    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public static Arm getInstance() {
        return instance;
    }

    public static Arm initialize(ArmIO io) {
        if (instance == null) {
            instance = new Arm(io, m_swerve, m_shoot);
        }
        return instance;
    }

    @RequiredArgsConstructor
    public enum Goal {
        STOP(() -> 0),
        AIM(() -> shootingAngle),
        AIM_NO_COMP(() -> shootingAngleNoComp),
        STOW(() -> 0),
        MANUAL(() -> 0),
        SUBWOOFER(() -> -5),
        PODIUM(() -> -22),
        PREPARE_CLIMB(() -> -10),
        PREPARE_TRAP(() -> -10),
        CLIMB(() -> 0),
        UNTRAP(() -> 0),
        SHUTTLE(() -> -12.5),
        CUSTOM(() -> -20);

        private final DoubleSupplier armSetpointSupplier;

        public double getDeg() {
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

    public Arm(ArmIO armIO, SwerveDrive m_swerve, Shoot m_shoot) {
        super();

        io = armIO;

        this.m_swerve = m_swerve;

        this.m_shoot = m_shoot;

        disableSupplier = DriverStation::isDisabled;

        pivotController = new ProfiledPIDController(ArmConstants.pivotkP,
                ArmConstants.pivotkI, ArmConstants.pivotkD,
                new Constraints(ArmConstants.kMaxAngularPivotSpeedDegreesPerSecond,
                        ArmConstants.kMaxAngularPivotAccelerationDegreesPerSecondSquared));

        pivotController.setIZone(1);

        pivotController.setTolerance(0.3);

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
            goal = Goal.MANUAL;
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

    public void updateShootAngleDeg() {
        shootingAngle = m_shoot.degToSpeaker();
    }

    public void updateShootAngleDegNoComp() {
        shootingAngleNoComp = m_shoot.degToSpeakerNoComp();
    }

    public void enableAutoShooting(boolean enable) {
        pivotSetpoint = shootingAngle;
        shooterPIDEnabled = enable;
    }

    @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
    public boolean atGoal() {
        return pivotController.atGoal();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        updateShootAngleDeg();
        updateShootAngleDegNoComp();

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