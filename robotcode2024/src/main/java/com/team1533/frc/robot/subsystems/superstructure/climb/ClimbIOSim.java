package com.team1533.frc.robot.subsystems.superstructure.climb;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.subsystems.superstructure.elevator.Elevator.IntakePosition;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO {
    private final ElevatorSim leftSim = new ElevatorSim(DCMotor.getNEO(1),
            Constants.ElevatorConstants.elevSimPosConv,
            Constants.ElevatorConstants.elevCarraigeKG, Constants.ElevatorConstants.elevDrumRadiusMeters,
            Units.inchesToMeters(Constants.ElevatorConstants.minHeightInches),
            Constants.ElevatorConstants.maxHeightInches, true, Constants.ElevatorConstants.minHeightInches);

}
