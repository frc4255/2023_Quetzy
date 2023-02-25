package frc.robot.commands.Arm;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class StowArm extends CommandBase {
  private final Arm m_Arm;
  private final Wrist m_Wrist;

  public StowArm(Arm m_Arm, Wrist m_Wrist) {
    this.m_Wrist = m_Wrist;
    this.m_Arm = m_Arm;
    addRequirements(m_Arm, m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.enable();
    m_Wrist.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.stow();
    m_Arm.stow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.disable();
    m_Wrist.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}