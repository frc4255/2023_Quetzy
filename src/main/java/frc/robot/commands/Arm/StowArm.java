package frc.robot.commands.Arm;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class StowArm extends CommandBase {
  private final Arm s_Arm;

  public StowArm(Arm s_Arm) {
    this.s_Arm = s_Arm;

    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Arm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.stow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_Arm.isNearGoal("stow")) {
      return true;
    } else {
      return false;
    }
  }
}