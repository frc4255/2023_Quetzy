package frc.robot.autos.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import com.pathplanner.lib.PathPlannerTrajectory;
public class ArmL3 extends CommandBase{
    
    private final Arm m_Arm;

    public ArmL3(Arm m_Arm) {
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
    m_Arm.setL3();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_Arm.isNearGoal("high")) {
      return true;
    } else {
      return false;
    }
  }
}
