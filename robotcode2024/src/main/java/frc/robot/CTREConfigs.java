package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfigurator;

public final class CTREConfigs {
    public TalonFXConfigurator swerveDriveFXConfig;

    public CTREConfigs() {
        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.driveEnableCurrentLimit,
                Constants.Swerve.driveContinuousCurrentLimit,
                Constants.Swerve.drivePeakCurrentLimit,
                Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;

    }
}
