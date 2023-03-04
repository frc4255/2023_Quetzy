package frc.robot.autos.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;

public class autoDock extends CommandBase{
    
    private final Swerve m_Swerve;
    private final RobotState s_RobotState;
    private Translation2d tilt;
    private PIDController rollPID = new PIDController(0.2, 0, 1);
    private PIDController pitchPID = new PIDController(0.2, 0, 1);

    public autoDock(Swerve m_Swerve, RobotState s_RobotState) {
        this.m_Swerve = m_Swerve;
        this.s_RobotState = s_RobotState;

        rollPID.setTolerance(2.5);
        pitchPID.setTolerance(2.5);
        addRequirements(m_Swerve);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_RobotState.toggleState(201);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt = new Translation2d(rollPID.calculate(m_Swerve.getRoll(), 0), pitchPID.calculate(m_Swerve.getPitch(), 0)); //TODO: robot might be backwards lol

    if (m_Swerve.getRoll() < 2.5 && m_Swerve.getPitch() < 2.5) {
      s_RobotState.toggleState(202);
    }

    m_Swerve.drive(tilt, 0, false, false);

    //Debugging stuff
    SmartDashboard.putNumber("X value (Roll)", tilt.getX());
    SmartDashboard.putNumber("Y Value (Pitch)", tilt.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Swerve.getRoll() < 2.5 && m_Swerve.getPitch() < 2.5) {
      return true;
    } else {
      return false;
    }
  }
}
