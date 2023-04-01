package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import edu.wpi.first.math.controller.ArmFeedforward;

public class Wrist extends ProfiledPIDSubsystem {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor;

  private ArmFeedforward m_feedforward;

  private RobotState s_RobotState;

  private double lastAbsoluteValue = 0.0;
  private double lastValueRadians = 0.0;
  private double incrementEncoderZero = 0.0;

  private boolean safety = false;
  private boolean encoderDisconnected = false;

  private enum wristPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF,
    SINGLE
  }

  private final HashMap<wristPositions, Double> cubeGoals = new HashMap<>();
  private final HashMap<wristPositions, Double> coneGoals = new HashMap<>();

  private HashMap<wristPositions, Double> goal;

  public Wrist(RobotState s_RobotState) {

    super(new ProfiledPIDController(
        Constants.Wrist.kP,
        Constants.Wrist.kI,
        Constants.Wrist.kD,
        new TrapezoidProfile.Constraints(Constants.Wrist.kMaxVelocityRads, Constants.Wrist.kMaxAccelerationRads))
    );

    this.s_RobotState = s_RobotState;

    //TODO: Set wrist cube goals
   cubeGoals.put(wristPositions.STOW, 4.38);
   cubeGoals.put(wristPositions.LOW, 3.1);
   cubeGoals.put(wristPositions.MID, 3.95);
   cubeGoals.put(wristPositions.HIGH, 3.8);
   cubeGoals.put(wristPositions.SHELF, 3.2);
   cubeGoals.put(wristPositions.SINGLE, 4.38);

   //TODO: Set wrist cone goals
   coneGoals.put(wristPositions.STOW, 4.38);
   coneGoals.put(wristPositions.LOW, 3.21);
   coneGoals.put(wristPositions.MID, 4.05);
   coneGoals.put(wristPositions.HIGH, 3.47);
   coneGoals.put(wristPositions.SHELF, 3.38);
   coneGoals.put(wristPositions.SINGLE, 4.38);

    encoder = new DutyCycleEncoder(0);
    encoder.setDistancePerRotation(2 * Math.PI);
    encoder.setPositionOffset(0.86);

    m_feedforward = new ArmFeedforward(Constants.Wrist.kS, Constants.Wrist.kG, Constants.Wrist.kV);

    motor = new WPI_TalonFX(25);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(true);
  }

  private void moveToPos(wristPositions pos) {

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
        setGoal(goal.get(wristPositions.LOW));
        break;
      case MID:
        setGoal(goal.get(wristPositions.MID));
        break;
      case HIGH:
        setGoal(goal.get(wristPositions.HIGH));
        break;
      case SHELF:
        setGoal(goal.get(wristPositions.SHELF));
        break;
      case STOW:
        setGoal(goal.get(wristPositions.STOW));
        break;
      case SINGLE:
        setGoal(goal.get(wristPositions.SINGLE));
        break;
    }
  }

  public boolean isNearGoal(String whatGoal) {

    double goalPos = 4.38;

    switch (whatGoal.toLowerCase()) {
      case "low":
        goalPos = goal.get(wristPositions.LOW);
        break;
      case "mid":
        goalPos = goal.get(wristPositions.MID);
        break;
      case "high":
        goalPos = goal.get(wristPositions.HIGH);
        break;
      case "shelf":
        goalPos = goal.get(wristPositions.SHELF);
        break;
      case "stow":
        goalPos = goal.get(wristPositions.STOW);
        break;
      case "single":
        goalPos = goal.get(wristPositions.SINGLE);
    } 

    if (Math.abs(getMeasurement() - goalPos) < 0.06 || getMeasurement() == goalPos) {
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

  public void setSingle() {
    moveToPos(wristPositions.SINGLE);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (passedZero()) {
      safety = true;
      System.out.println("FATAL ERROR: WRIST HAS PASSED ENCODER ZERO");
    }

    lastValueRadians = getMeasurement();
    lastAbsoluteValue = encoder.getAbsolutePosition();

    if (checkEncoderConnection()) {
      encoderDisconnected = true;
      motor.stopMotor();
      System.out.println("FATAL ERROR: WRIST HAS LOST ENCODER CONNECTION");
    }

    SmartDashboard.putNumber("Wrist angle", getMeasurement());
    SmartDashboard.putNumber("Wrist absolute position", encoder.getAbsolutePosition());
  }

  @Override
  protected void useOutput(double output, State tempsetpoint) {
    double feedforward = m_feedforward.calculate(tempsetpoint.position, tempsetpoint.velocity);
    motor.setVoltage(output + feedforward);
  }

  @Override
  protected double getMeasurement() {
    return (encoder.getDistance() + incrementEncoderZero) * (-1);
  }

  @Override
  public void simulationPeriodic() {
  }

  private boolean passedZero() {
    if (Math.abs(encoder.getAbsolutePosition() - lastAbsoluteValue) > 0.9) {
      if (encoder.getAbsolutePosition() < 0.1) {
        incrementEncoderZero = lastValueRadians;
      } else if (encoder.getAbsolutePosition() > 0.9) {
        incrementEncoderZero = -lastValueRadians;
      }
      return true;
    } else {
      return false;
    }
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