package com.team1533.frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.util.AbsoluteEncoder;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleIOReal implements ModuleIO {
    // motors
    private final TalonFX m_driveMotor;
    private final CANSparkMax m_turningMotor;

    // encoders
    final RelativeEncoder m_turningEncoder;
    final AbsoluteEncoder m_absoluteEncoder;

    // final AbsoluteEncoder absoluteEncoder;

    public DutyCycleOut output;

    // steering pid
    private SparkPIDController m_pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public ModuleIOReal(int driveMotorChannel, int turningMotorChannel, int absoluteEncoderChannel,
            boolean turningEncoderReversed, double angleOffset) {

        output = new DutyCycleOut(0);

        output.UpdateFreqHz = 0;

        output.EnableFOC = true;

        m_driveMotor = new TalonFX(driveMotorChannel);
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        m_turningMotor.restoreFactoryDefaults();

        m_turningEncoder = m_turningMotor.getEncoder();
        m_absoluteEncoder = new AbsoluteEncoder(absoluteEncoderChannel, angleOffset);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution (ratio of distance traveled by wheel to distance traveled by
        // encoder shaft)

        // Set whether drive encoder should be reversed or not

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.

        m_driveMotor.getConfigurator()
                .apply(new FeedbackConfigs()
                        .withSensorToMechanismRatio(1.0 / ModuleConstants.kDriveEncoderDistancePerPulse));

        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kSteerEncoderDistancePerPulse);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderDistancePerPulse / 60.);
        // m_absoluteEncoder.setPositionConversionFactor(encoderCPR);

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

        m_pidController = m_turningMotor.getPIDController();
        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        m_driveMotor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.ELECTRICAL.swerveDrivingCurrentLimit)
                .withSupplyCurrentLimitEnable(true));
        m_turningMotor.setSmartCurrentLimit(Constants.ELECTRICAL.swerveTurningCurrentLimit);
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
        m_turningMotor.setIdleMode(IdleMode.kBrake);
        m_turningMotor.setInverted(true);
        m_driveMotor.setInverted(true);
        m_driveMotor.setControl(output);

        m_turningMotor.burnFlash();

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

    }

    @Override
    public void runDriveVolts(double volts) {
    }

    @Override
    public void runTurnVolts(double volts) {
    }

    @Override
    public void runCharacterization(double input) {
    }

    @Override
    public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {

    }

    @Override
    public void runTurnPositionSetpoint(double angleRads) {
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
            output.Output = 0;
            m_driveMotor.setControl(output);
            return;
        }
        double desiredSteering = state.angle.getRadians();
        double currentSteering = m_turningEncoder.getPosition();

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
        output.Output = desiredDrive;
        m_driveMotor.setControl(output);
        m_pidController.setReference(steeringSetpoint, com.revrobotics.CANSparkBase.ControlType.kPosition);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {

    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {

    }

    @Override
    public void setDriveBrakeMode(boolean enable) {

    }

    @Override
    public void setTurnBrakeMode(boolean enable) {

    }

    @Override
    public void stop() {

    }
}