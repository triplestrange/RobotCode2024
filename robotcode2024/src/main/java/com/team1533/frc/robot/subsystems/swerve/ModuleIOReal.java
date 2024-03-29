package com.team1533.frc.robot.subsystems.swerve;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.util.AbsoluteEncoder;
import com.team1533.lib.swerve.ModuleConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class ModuleIOReal implements ModuleIO {
    // motors
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    // encoders
    private RelativeEncoder turningEncoder;
    private AbsoluteEncoder absoluteEncoder;

    // Status Signals
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTorqueCurrent;

    // steering pid
    private SparkPIDController pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    // Control
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0)
            .withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);

    public ModuleIOReal(ModuleConfig config) {

        // Init motor & encoder objects
        driveMotor = new TalonFX(config.getDriveMotorChannel());
        turningMotor = new CANSparkMax(config.getTurningMotorChannel(), MotorType.kBrushless);
        absoluteEncoder = new AbsoluteEncoder(config.getAbsoluteEncoderChannel(), config.getAngleOffset());
        turningEncoder = turningMotor.getEncoder();

        turningMotor.restoreFactoryDefaults();

        turningMotor.restoreFactoryDefaults();
        turningMotor.setCANTimeout(250);

        // turningMotor.enableVoltageCompensation(12.0);

        turningEncoder.setPosition(0.0);
        turningEncoder.setMeasurementPeriod(10);
        turningEncoder.setAverageDepth(2);

        turningEncoder = turningMotor.getEncoder();

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution (ratio of distance traveled by wheel to distance traveled by
        // encoder shaft)

        // Set whether drive encoder should be reversed or not

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.

        driveMotor.getConfigurator()
                .apply(new FeedbackConfigs()
                        .withSensorToMechanismRatio(1.0 / ModuleConstants.kDriveEncoderDistancePerPulse));

        turningEncoder.setPositionConversionFactor(ModuleConstants.kSteerEncoderDistancePerPulse);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderDistancePerPulse / 60.);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        // m_turningPIDController.enableContinuousInput(0, 2*Math.PI);

        // PID coefficients
        kP = 2; // 0.5
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        pidController = turningMotor.getPIDController();
        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.ELECTRICAL.swerveDrivingCurrentLimit)
                .withSupplyCurrentLimitEnable(true));
        turningMotor.setSmartCurrentLimit(Constants.ELECTRICAL.swerveTurningCurrentLimit);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        turningMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setInverted(true);
        driveMotor.setInverted(true);
        driveMotor.setControl(velocityTorqueCurrentFOC);

        turningMotor.burnFlash();

        // Get signals and set update rate
        // 100hz signals
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTorqueCurrent = driveMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
                .isOK();
        inputs.turnMotorConnected = turningMotor.getFault(FaultID.kSensorFault);
        inputs.absMotorConnected = absoluteEncoder.isConnected();
        inputs.hasCurrentControl = true;

        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
        inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

        inputs.turnAbsolutePosition = new Rotation2d(absoluteEncoder.getAngle());
        inputs.turnPosition = new Rotation2d().fromRotations(turningEncoder.getPosition());
        inputs.turnVelocityPerSec = new Rotation2d().fromRotations(turningEncoder.getVelocity());
        inputs.turnAppliedVolts = turningMotor.getAppliedOutput() * turningMotor.getBusVoltage();
        inputs.turnSupplyCurrentAmps = turningMotor.getOutputCurrent();
    }

    @Override
    public void runDriveVolts(double volts) {
        driveMotor.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
    }

    @Override
    public void runTurnVolts(double volts) {
        turningMotor.setVoltage(volts);
    }

    @Override
    public void runCharacterization(double input) {
        driveMotor.setControl(currentControl.withOutput(input));
    }

    @Override
    public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
        driveMotor.setControl(velocityTorqueCurrentFOC.withVelocity(Units.radiansToRotations(velocityRadsPerSec))
                .withFeedForward(feedForward));
    }

    @Override
    public void runTurnPositionSetpoint(double angleRads) {
        pidController.setReference(Units.radiansToRotations(angleRads), ControlType.kPosition);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, false);
    }

    @Override
    public void setDesiredState(SwerveModuleState state, boolean forceAngle) {

        double desiredDrive = state.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;

        if (Math.abs(desiredDrive) < 0.01 && !forceAngle) {
            velocityTorqueCurrentFOC.Velocity = 0;
            driveMotor.setControl(velocityTorqueCurrentFOC);
            return;
        }
        double desiredSteering = state.angle.getRadians();
        double currentSteering = turningEncoder.getPosition();

        // calculate shortest path to angle with forward drive (error -pi to pi)
        double steeringError = Math.IEEEremainder(desiredSteering - currentSteering, 2 * Math.PI);

        // reverse drive if error is larger than 90 degrees
        if (steeringError > Math.PI / 2) {
            steeringError -= Math.PI;
            desiredDrive *= -1;
        } else if (steeringError < -Math.PI / 2) {
            steeringError += Math.PI;
            desiredDrive *= -1;
        }

        double steeringSetpoint = currentSteering + steeringError;

        // m_driveMotor.set(desiredDrive + Math.cos(steeringError));
        velocityTorqueCurrentFOC.Velocity = desiredDrive;
        driveMotor.setControl(velocityTorqueCurrentFOC);
        pidController.setReference(steeringSetpoint, com.revrobotics.CANSparkBase.ControlType.kPosition);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
    }

    @Override
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningEncoder.setPosition(absoluteEncoder.getAngle());
    }

    @Override
    public void setDriveBrakeMode(NeutralModeValue neutralModeValue) {
        driveMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(neutralModeValue));
    }

    @Override
    public void setTurnBrakeMode(IdleMode idleMode) {
        turningMotor.setIdleMode(idleMode);
    }

    @Override
    public void stop() {
        driveMotor.setControl(neutralControl);
        turningMotor.stopMotor();
    }
}