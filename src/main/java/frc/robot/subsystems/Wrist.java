package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class Wrist extends ProfiledPIDSubsystem {

  private DutyCycleEncoder encoder;
  private WPI_TalonFX motor;

  private double setpoint;

  private ArmFeedforward m_feedforward;

  private enum wristPositions {
    STOW,
    LOW,
    MID,
    HIGH,
    SHELF
  }

  private final HashMap<wristPositions, Double> wristSetpoints = new HashMap<>();

  /** Creates a new Arm Subsystem. */
  public Wrist() {

    // TODO: Deploy code to robot and find setpoint values by moving the wrist and checking shuffleboard, then run SysID and pray.

    super(new ProfiledPIDController(
      Constants.Wrist.kP,
      Constants.Wrist.kI,
      Constants.Wrist.kD,
      new TrapezoidProfile.Constraints(Constants.Wrist.kMaxVelocityRads, Constants.Wrist.kMaxAccelerationRads))
    );

    wristSetpoints.put(wristPositions.STOW, 4.38); // TODO: Update STOW Setpoint
    wristSetpoints.put(wristPositions.LOW, 3.12); // TODO: Update LOW Setpoint
    wristSetpoints.put(wristPositions.MID, 3.95); // TODO: Update MID Setpoint
    wristSetpoints.put(wristPositions.HIGH, 3.45); // TODO: Update HIGH Setpoint
    wristSetpoints.put(wristPositions.SHELF, 3.2); // TODO: Update SHELF Setpoint

    encoder = new DutyCycleEncoder(0); // TODO: Ensure encoder object has correct DIO channel
    encoder.setDistancePerRotation(2*Math.PI);
    encoder.setPositionOffset(0.1);

    m_feedforward = new ArmFeedforward(Constants.Wrist.kS, Constants.Wrist.kG, Constants.Wrist.kV);

    motor = new WPI_TalonFX(25); // TODO: Update Motor ID
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(true);
  }

  public void reset() {
    encoder.reset();
  }

  public void moveUp() {
    motor.set(ControlMode.PercentOutput, 0.15);
  }

  public void moveDown() {
    motor.set(ControlMode.PercentOutput, -0.15);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  private void moveToPos(wristPositions pos) {
    switch (pos) {
      case LOW:
        setGoal(wristSetpoints.get(wristPositions.LOW));
        break;
      case MID:
        setGoal(wristSetpoints.get(wristPositions.MID));
        break;
      case HIGH:
        setGoal(wristSetpoints.get(wristPositions.HIGH));
        break;
      case SHELF:
        setGoal(wristSetpoints.get(wristPositions.SHELF));
        break;
      case STOW:
        setGoal(wristSetpoints.get(wristPositions.STOW));
        break;
    }
  }

  public double getPos() {
    return encoder.getAbsolutePosition();
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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder value", encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Wrist position", encoder.getDistance() * -1 + 2*Math.PI);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  protected void useOutput(double output, State tempsetpoint) {
    double feedforward = m_feedforward.calculate(tempsetpoint.position, tempsetpoint.velocity);
    motor.setVoltage(output + feedforward);
    SmartDashboard.putNumber("PID voltage", output);
    SmartDashboard.putNumber("Setpoint position", tempsetpoint.position);
    SmartDashboard.putNumber("Setpoint velocity", tempsetpoint.velocity);
  }

  @Override
  protected double getMeasurement() {
    return encoder.getDistance() * -1 + 2*Math.PI;
  }
}