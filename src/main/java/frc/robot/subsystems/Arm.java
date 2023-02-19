package frc.robot.subsystems;

import java.util.HashMap;

import org.opencv.core.Mat;

import java.io.ObjectOutputStream.PutField;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm extends ProfiledPIDSubsystem {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;

  //private double goal;
  private TrapezoidProfile.State setpoint;

  private PIDController pid;
  private ArmFeedforward m_feedforward;

  private final Double topLimit = 1000.0;
  private final Double bottomLimit = -1000.0;
  private enum armPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF
  }

  private final HashMap<armPositions, Double> armgoals = new HashMap<>();

  /** Creates a new Arm Subsystem. */
  public Arm() {

    // TODO: Deploy code to robot and find goal values by moving the arm and checking shuffleboard, then run SysID and pray.

    super(new ProfiledPIDController(
      Constants.Arm.kP,
      Constants.Arm.kI,
      Constants.Arm.kD,
      new TrapezoidProfile.Constraints(Constants.Arm.kMaxVelocityRads, Constants.Arm.kMaxAccelerationRads))
    );

    armgoals.put(armPositions.STOW, 1.6); // TODO: Update STOW goal
    armgoals.put(armPositions.LOW, 1.68); // TODO: Update LOW goal
    armgoals.put(armPositions.MID, 2.07); // TODO: Update MID goal
    armgoals.put(armPositions.HIGH, 3.4); // TODO: Update HIGH goal
    armgoals.put(armPositions.SHELF, 3.4); // TODO: Update SHELF goal

    encoder = new DutyCycleEncoder(1); // TODO: Ensure encoder object has correct DIO channel
    encoder.setDistancePerRotation(2*Math.PI);
    encoder.setPositionOffset(0.5);

    //pid = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
    m_feedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

    motor1 = new WPI_TalonFX(20); // TODO: Update Motor ID
    motor2 = new WPI_TalonFX(21); // TODO: Update Motor ID
    motor1.setInverted(true);
    motor2.setInverted(true);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);
    super.enable();
    setpoint = new TrapezoidProfile.State(1.7, 0);
  }

  private void moveToPos(armPositions pos) {
    switch (pos) {
      case LOW:
        setpoint = setPos(armgoals.get(armPositions.LOW));
        break;
      case MID:
        setpoint = setPos(armgoals.get(armPositions.MID));
        break;
      case HIGH:
        setpoint = setPos(armgoals.get(armPositions.HIGH));
        break;
      case SHELF:
        setpoint = setPos(armgoals.get(armPositions.SHELF));
        break;
      case STOW:
        setpoint = setPos(armgoals.get(armPositions.STOW));
        break;
    }
    m_controller.setGoal(setpoint);
  }

  public void setL1() {
    moveToPos(armPositions.LOW);
  }

  public void setL2() {
    moveToPos(armPositions.MID);
    System.out.println("HEllo");
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

  public void resetPID() {
    pid.reset();  
  }

  public void moveUp() {
    motor1.set(ControlMode.PercentOutput, 0.25);
    motor2.set(ControlMode.PercentOutput, 0.25);
  }

  public void moveDown() {
    motor1.set(ControlMode.PercentOutput, -0.25);
    motor2.set(ControlMode.PercentOutput, -0.25);
  }

  private TrapezoidProfile.State setPos(double goal) {
    return new TrapezoidProfile.State(goal, 0);
  }
  

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    motor1.setVoltage(output + feedforward);
    motor2.setVoltage(output + feedforward);
    System.out.println(feedforward);
    SmartDashboard.putNumber("Setpoint position", setpoint.position);
    SmartDashboard.putNumber("Setpoint velocity", setpoint.velocity);
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Arm encoder value", encoder.getDistance());
  }

  @Override
  protected double getMeasurement() {
    return encoder.getDistance();
  }
}