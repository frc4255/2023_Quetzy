package frc.robot.subsystems;

import java.util.HashMap;

import org.opencv.core.Mat;

import java.io.ObjectOutputStream.PutField;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm extends SubsystemBase {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;

  private double setpoint;

  private PIDController pid;
  private ArmFeedforward feedforward;

  private final Double topLimit = 1000.0;
  private final Double bottomLimit = -1000.0;
  private enum armPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF
  }

  private final HashMap<armPositions, Double> armSetpoints = new HashMap<>();

  /** Creates a new Arm Subsystem. */
  public Arm() {

    // TODO: Deploy code to robot and find setpoint values by moving the arm and checking shuffleboard, then run SysID and pray.

    armSetpoints.put(armPositions.STOW, 1.6); // TODO: Update STOW Setpoint
    armSetpoints.put(armPositions.LOW, 1.68); // TODO: Update LOW Setpoint
    armSetpoints.put(armPositions.MID, 2.07); // TODO: Update MID Setpoint
    armSetpoints.put(armPositions.HIGH, 3.4); // TODO: Update HIGH Setpoint
    armSetpoints.put(armPositions.SHELF, 3.4); // TODO: Update SHELF Setpoint

    encoder = new DutyCycleEncoder(1); // TODO: Ensure encoder object has correct DIO channel
    encoder.setDistancePerRotation(2*Math.PI);
    encoder.setPositionOffset(0.5);

    pid = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
    feedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

    motor1 = new WPI_TalonFX(20); // TODO: Update Motor ID
    motor2 = new WPI_TalonFX(21); // TODO: Update Motor ID
    motor1.setInverted(true);
    motor2.setInverted(true);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);
  }

  public void reset() {
    encoder.reset();
  }

  public void stop() {
    motor1.set(ControlMode.PercentOutput, 0.0);
    motor2.set(ControlMode.PercentOutput, 0.0);
  }

  private double getPosInRadians() {
   return (encoder.getDistance()) * 2 * Math.PI;
  }
  private void moveToPos(armPositions pos) {
    switch (pos) {
      case LOW:
        setpoint = armSetpoints.get(armPositions.LOW);
        break;
      case MID:
        setpoint = armSetpoints.get(armPositions.MID);
        break;
      case HIGH:
        setpoint = armSetpoints.get(armPositions.HIGH);
        break;
      case SHELF:
        setpoint = armSetpoints.get(armPositions.SHELF);
        break;
      case STOW:
        setpoint = armSetpoints.get(armPositions.STOW);
        break;
    }

    if (isAtLimit()) {
      return;
    }


    motor1.setVoltage(pid.calculate(encoder.getDistance(), setpoint) + feedforward.calculate(setpoint, 0)*-1); //TODO: Figure out feedforward method
    motor2.setVoltage(pid.calculate(encoder.getDistance(), setpoint) + feedforward.calculate(setpoint, 0)*-1); //TODO: Figure out feedforward method

  }

  public double getPos() {
    return encoder.getAbsolutePosition();
  }

  private Boolean isAtLimit() {
    if (getPos() >= topLimit || getPos() <= bottomLimit) {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder value", encoder.getDistance());
    SmartDashboard.putNumber("Arm Pos in Radians", getPosInRadians());
    SmartDashboard.putNumber("feedfoward input", getPosInRadians() - (Math.PI/2));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}