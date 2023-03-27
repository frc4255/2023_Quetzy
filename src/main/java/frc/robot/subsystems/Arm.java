package frc.robot.subsystems;

import java.util.HashMap;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm extends ProfiledPIDSubsystem {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;

  private ArmFeedforward m_feedforward;

  private RobotState s_RobotState;

  private enum armPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF
  }

  private final HashMap<armPositions, Double> coneGoals = new HashMap<>();
  private final HashMap<armPositions, Double> cubeGoals = new HashMap<>();
  private HashMap<armPositions, Double> goal;

  public Arm(RobotState s_RobotState) {
    super(new ProfiledPIDController(
        Constants.Arm.kP,
        Constants.Arm.kI,
        Constants.Arm.kD,
        new TrapezoidProfile.Constraints(Constants.Arm.kMaxVelocityRads, Constants.Arm.kMaxAccelerationRads)));

    this.s_RobotState = s_RobotState;

    //TODO: Set arm cone goals
    coneGoals.put(armPositions.STOW, 1.6);
    coneGoals.put(armPositions.LOW, 1.68);
    coneGoals.put(armPositions.MID, 2.2);
    coneGoals.put(armPositions.HIGH, 3.4);
    coneGoals.put(armPositions.SHELF, 3.4);

    //TODO: Set arm cube goals
    cubeGoals.put(armPositions.STOW, 1.6);
    cubeGoals.put(armPositions.LOW, 1.68);
    cubeGoals.put(armPositions.MID, 2.4);
    cubeGoals.put(armPositions.HIGH, 3.1);
    cubeGoals.put(armPositions.SHELF, 3.4)
    ;
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

    if (s_RobotState.getCurrentState() == RobotState.State.CUBE) {
      goal = cubeGoals;
    } else {
      goal = coneGoals;
    }

    if (!encoder.isConnected()) {
      s_RobotState.setState(RobotState.State.ENCODER_DISCONNECTED);;
      return;
    }

      switch (pos) {
        case LOW:
          setGoal(goal.get(armPositions.LOW));
          break;
        case MID:
          setGoal(goal.get(armPositions.MID));
          break;
        case HIGH:
          setGoal(goal.get(armPositions.HIGH));
          break;
        case SHELF:
          setGoal(goal.get(armPositions.SHELF));
          break;
        case STOW:
          setGoal(goal.get(armPositions.STOW));
          break;
      }

  }

  public boolean isNearGoal(String whatgoal) {

    double goalPos = 1.6;

    switch (whatgoal.toLowerCase()) {
      case "low":
        goalPos = goal.get(armPositions.LOW);
        break;
      case "mid":
        goalPos = goal.get(armPositions.MID);
        break;
      case "high":
        goalPos = goal.get(armPositions.HIGH);
        break;
      case "shelf":
        goalPos = goal.get(armPositions.SHELF);
        break;
      case "stow":
        goalPos = goal.get(armPositions.STOW);
        break;
    }

    if (Math.abs(encoder.getDistance() - goalPos) < 0.06 || encoder.getDistance() == goalPos) {
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

    SmartDashboard.putNumber("Arm angle", encoder.getDistance());
    SmartDashboard.putNumber("Arm error", getController().getPositionError());
  }

  @Override
  public double getMeasurement() {
    return encoder.getDistance();
  }
}