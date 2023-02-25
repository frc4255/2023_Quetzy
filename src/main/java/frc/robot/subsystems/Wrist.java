package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Wrist extends ProfiledPIDSubsystem {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor;

  private ArmFeedforward m_feedforward;

  private enum wristPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF
  }

  private final HashMap<wristPositions, Double> wristGoals = new HashMap<>();

  public Wrist() {

    super(new ProfiledPIDController(
        Constants.Wrist.kP,
        Constants.Wrist.kI,
        Constants.Wrist.kD,
        new TrapezoidProfile.Constraints(Constants.Wrist.kMaxVelocityRads, Constants.Wrist.kMaxAccelerationRads))
    );

   wristGoals.put(wristPositions.STOW, 4.38);
   wristGoals.put(wristPositions.LOW, 3.12);
   wristGoals.put(wristPositions.MID, 3.95);
   wristGoals.put(wristPositions.HIGH, 3.45);
   wristGoals.put(wristPositions.SHELF, 3.2);

    encoder = new DutyCycleEncoder(0);
    encoder.setDistancePerRotation(2 * Math.PI);
    encoder.setPositionOffset(0.1);

    m_feedforward = new ArmFeedforward(Constants.Wrist.kS, Constants.Wrist.kG, Constants.Wrist.kV);

    motor = new WPI_TalonFX(25);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(true);
  }

  private void moveToPos(wristPositions pos) {
    switch (pos) {
      case LOW:
        setGoal(wristGoals.get(wristPositions.LOW));
        break;
      case MID:
        setGoal(wristGoals.get(wristPositions.MID));
        break;
      case HIGH:
        setGoal(wristGoals.get(wristPositions.HIGH));
        break;
      case SHELF:
        setGoal(wristGoals.get(wristPositions.SHELF));
        break;
      case STOW:
        setGoal(wristGoals.get(wristPositions.STOW));
        break;
    }
  }

  public boolean isNearGoal(String goal) {

    double goalPos = 4.38;

    switch (goal.toLowerCase()) {
      case "low":
        goalPos = wristGoals.get(wristPositions.LOW);
        break;
      case "mid":
        goalPos = wristGoals.get(wristPositions.MID);
        break;
      case "high":
        goalPos = wristGoals.get(wristPositions.HIGH);
        break;
      case "shelf":
        goalPos = wristGoals.get(wristPositions.SHELF);
        break;
      case "stow":
        goalPos = wristGoals.get(wristPositions.STOW);
        break;
    } 

    if (Math.abs(getMeasurement() - goalPos) < 0.04 || getMeasurement() == goalPos) {
      return true;
    } else {
      return false;
    }
  }

  public void setL1() {
    moveToPos(wristPositions.LOW);
  }

  public void setL2() {
    moveToPos(wristPositions.MID);
  }

  public void setL3() {
    moveToPos(wristPositions.HIGH);
  }

  public void setShelf() {
    moveToPos(wristPositions.SHELF);
  }

  public void stow() {
    moveToPos(wristPositions.STOW);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  protected void useOutput(double output, State tempsetpoint) {
    double feedforward = m_feedforward.calculate(tempsetpoint.position, tempsetpoint.velocity);
    motor.setVoltage(output + feedforward);
  }

  @Override
  protected double getMeasurement() {
    return encoder.getDistance() * -1 + 2 * Math.PI;
  }

  @Override
  public void simulationPeriodic() {
  }
}