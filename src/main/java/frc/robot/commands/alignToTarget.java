package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.RobotState.robotStates;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Limelight;

public class alignToTarget extends CommandBase{
    
    private final Swerve s_Swerve;
    private final RobotContainer m_robotContainer;
    private final RobotState s_RobotState;
    private final Limelight leftLimelight;
    private final Limelight rightLimelight;
    private PIDController rotPID;
    private PIDController strafePID;

    public alignToTarget(Swerve s_Swerve, RobotContainer m_robotContainer, RobotState s_RobotState, Limelight leftLimelight, Limelight rightLimelight) {
        this.s_Swerve = s_Swerve;
        this.m_robotContainer = m_robotContainer;
        this.s_RobotState = s_RobotState;
        this.leftLimelight = leftLimelight;
        this.rightLimelight = rightLimelight;

        rotPID = new PIDController(0, 0, 0); //TODO: Tune align rotation PID
        strafePID = new PIDController(0, 0, 0); //TODO: Tune strafe PID

        strafePID.setTolerance(0.5);

        addRequirements(s_Swerve);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftLLx = 0.0;
    double rightLLx = 0.0;

    if (leftLimelight.hasTarget()) {
      leftLLx = leftLimelight.getTx();
    }

    if (rightLimelight.hasTarget()) {
      rightLLx = rightLimelight.getTx();
    }

    double avgX = (leftLLx + rightLLx) / 2;

    Translation2d values = new Translation2d(strafePID.calculate(avgX, 0.0), 0);
    //Rotation2d rotvalues = new Rotation2d(rotPID.calculate(s_Swerve.getYaw(), 0));

    s_Swerve.drive(values, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (strafePID.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}
