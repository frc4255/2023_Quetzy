package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class moveArmUp extends CommandBase{
    
    private final Arm m_Arm;

    public moveArmUp(Arm m_Arm) {
        this.m_Arm = m_Arm;

        addRequirements(m_Arm);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.moveUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}