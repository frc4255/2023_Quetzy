package frc.robot.autos.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
public class autoDock extends CommandBase{
    
    private final Swerve m_Swerve;
    private Translation2d tilt;

    public autoDock(Swerve m_Swerve) {
        this.m_Swerve = m_Swerve;

        addRequirements(m_Swerve);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt = new Translation2d(m_Swerve.getRoll(), m_Swerve.getPitch()); // TODO: switch them?? 

    m_Swerve.drive(tilt.times(0.2), 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
