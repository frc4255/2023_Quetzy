package frc.robot.subsystems;

import java.util.HashMap;
import java.lang.Math;


import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm extends ProfiledPIDSubsystem {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;

  private ArmFeedforward m_feedforward;

  private RobotState s_RobotState;

  private double lastValueRadians = 0.0;
  private double incrementEncoderZero = 0.0;
  private boolean safety = false;
  private boolean encoderDisconnected = false;

  private DataLog log;
  private BooleanLogEntry encoderConnectionLog;
  private DoubleLogEntry encoderIncrementLog;

  private enum armPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF,
    SINGLE
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

    coneGoals.put(armPositions.STOW, -1.44);
    coneGoals.put(armPositions.LOW, -1.39);
    coneGoals.put(armPositions.MID, -0.89);
    coneGoals.put(armPositions.HIGH, 0.38);
    coneGoals.put(armPositions.SHELF, 0.48);
    coneGoals.put(armPositions.SINGLE, -0.87);

    cubeGoals.put(armPositions.STOW, -1.44);
    cubeGoals.put(armPositions.LOW, -1.39);
    cubeGoals.put(armPositions.MID, -0.7);
    cubeGoals.put(armPositions.HIGH, 0.15);
    cubeGoals.put(armPositions.SHELF, 0.16);
    cubeGoals.put(armPositions.SINGLE, -0.67);
    ;
    encoder = new DutyCycleEncoder(1);
    encoder.setDistancePerRotation(2 * Math.PI);
    encoder.setPositionOffset(0.51);

    m_feedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

    motor1 = new WPI_TalonFX(20);
    motor2 = new WPI_TalonFX(21);
    motor1.setInverted(true);
    motor2.setInverted(true);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

    /* Telemetry */
    log = DataLogManager.getLog();

    encoderConnectionLog = new BooleanLogEntry(log, "/arm");
    encoderIncrementLog = new DoubleLogEntry(log, "/arm");
  }

  private void moveToPos(armPositions pos) {

    if (s_RobotState.getCurrentState() == RobotState.State.CUBE) {
      goal = cubeGoals;
    } else {
      goal = coneGoals;
    }

    if (runSafetyChecks()) {
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
      case SINGLE:
        setGoal(goal.get(armPositions.SINGLE));
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
      case "single":
        goalPos = goal.get(armPositions.SINGLE);
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

  public void setSingle() {
    moveToPos(armPositions.SINGLE);
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

    checkForRollover();

    lastValueRadians = getMeasurement();

    if (checkEncoderConnection()) {
      encoderDisconnected = true;
      disable();
      motor1.stopMotor();
      motor2.stopMotor();
      DataLogManager.log("ARM ENCODER DISCONNECTION");
      encoderConnectionLog.append(false);
    } else {
      encoderConnectionLog.append(true);
    }

    SmartDashboard.putNumber("Arm angle", getMeasurement());
    SmartDashboard.putNumber("Arm error", getController().getPositionError());
    SmartDashboard.putNumber("Arm Absolute Position", encoder.getAbsolutePosition());
  }

  @Override
  public double getMeasurement() {
    return encoder.getDistance() + incrementEncoderZero;
  }

  /*
   * Rollover Protection
   */
  private void checkForRollover() {
    double delta = getMeasurement() - lastValueRadians;

    if (delta > 0.9 * (2 * Math.PI)) {
      incrementEncoderZero += (2 * Math.PI);
    } else if (delta < -0.9 * (2 * Math.PI)) {
      incrementEncoderZero -= (2 * Math.PI);
    }
    encoderIncrementLog.append(incrementEncoderZero);
  }

  private boolean checkEncoderConnection() {
    if (!encoder.isConnected()) {
      s_RobotState.setState(RobotState.State.ENCODER_DISCONNECTED);
      return true;
    }
    return false;
  }

  private boolean runSafetyChecks() {
    if (safety || encoderDisconnected) {
      return true;
    }

    return false;
  }
}