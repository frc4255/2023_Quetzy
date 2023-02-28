package frc.robot.subsystems;

import java.util.HashMap;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm extends ProfiledPIDSubsystem {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;

  private ArmFeedforward m_feedforward;

  private enum armPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF
  }

  private final HashMap<armPositions, Double> armgoals = new HashMap<>();

  public Arm() {
    super(new ProfiledPIDController(
        Constants.Arm.kP,
        Constants.Arm.kI,
        Constants.Arm.kD,
        new TrapezoidProfile.Constraints(Constants.Arm.kMaxVelocityRads, Constants.Arm.kMaxAccelerationRads))
    );

    armgoals.put(armPositions.STOW, 1.6);
    armgoals.put(armPositions.LOW, 1.68);
    armgoals.put(armPositions.MID, 2.07);
    armgoals.put(armPositions.HIGH, 3.4);
    armgoals.put(armPositions.SHELF, 3.4);

    encoder = new DutyCycleEncoder(1);
    encoder.setDistancePerRotation(2 * Math.PI);
    encoder.setPositionOffset(0.5);

    m_feedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

    motor1 = new WPI_TalonFX(20);
    motor2 = new WPI_TalonFX(21);
    motor1.setInverted(true);
    motor2.setInverted(true);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);
  }

  private void moveToPos(armPositions pos) {
    switch (pos) {
      case LOW:
        setGoal(1.68);
        break;
      case MID:
        setGoal(2.1);
        break;
      case HIGH:
        setGoal(3.4);
        break;
      case SHELF:
        setGoal(3.4);
        break;
      case STOW:
        setGoal(1.6);
        break;
    }
  }

  public boolean isNearGoal(String goal) {

    double goalPos = 1.6;

    switch (goal.toLowerCase()) {
      case "low":
        goalPos = armgoals.get(armPositions.LOW);
        break;
      case "mid":
        goalPos = armgoals.get(armPositions.MID);
        break;
      case "high":
        goalPos = armgoals.get(armPositions.HIGH);
        break;
      case "shelf":
        goalPos = armgoals.get(armPositions.SHELF);
        break;
      case "stow":
        goalPos = armgoals.get(armPositions.STOW);
        break;
    }

    if (Math.abs(encoder.getDistance() - goalPos) < 0.04 || encoder.getDistance() == goalPos) {
      return true;
    } else {
      return false;
    }
  }
  public void setL1() {
    moveToPos(armPositions.LOW);
  }

  public void setL2() {
    moveToPos(armPositions.MID);
  }

  public void setL3() {
    moveToPos(armPositions.HIGH);
  }

  public void setShelf() {
    moveToPos(armPositions.SHELF);
  }

  public void stow() {
    moveToPos(armPositions.STOW);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    motor1.setVoltage(output + feedforward);
    motor2.setVoltage(output + feedforward);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public double getMeasurement() {
    return encoder.getDistance();
  }
}