package frc.robot.commands.Arm;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class MidArm extends CommandBase {
  private final Arm m_Arm;

  public MidArm(Arm m_Arm) {
    this.m_Arm = m_Arm;

    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setL2();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}