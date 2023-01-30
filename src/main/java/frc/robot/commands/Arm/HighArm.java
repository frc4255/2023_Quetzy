package frc.robot.commands.Arm;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class HighArm extends CommandBase {
  private final Arm m_Arm;

  public HighArm(Arm m_Arm) {
    this.m_Arm = m_Arm;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setL3();
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