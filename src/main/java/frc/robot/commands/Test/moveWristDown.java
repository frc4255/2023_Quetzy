package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class moveWristDown extends CommandBase{
    
    private final Wrist m_Wrist;

    public moveWristDown(Wrist m_Wrist) {
        this.m_Wrist = m_Wrist;

        addRequirements(m_Wrist);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.moveDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}