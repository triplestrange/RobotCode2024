package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.cannon.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.cannon.Shooter;

public class GroundToConveyor extends Command {
    private Conveyor m_Conveyor;
    private Intake m_Intake;
    /**
     * Creates a new Drive.
     * 
     * normal = 2.5
     * slow = 0.75
     */
    public GroundToConveyor(Conveyor m_Conveyor, Intake m_Intake) {
        addRequirements(m_Conveyor, m_Intake);
        this.m_Conveyor = m_Conveyor;
        this.m_Intake = m_Intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Conveyor.runConvIn();
        m_Intake.runIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Conveyor.conveyorOff();
        m_Intake.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Conveyor.getConveyorSensor();
    }
}
