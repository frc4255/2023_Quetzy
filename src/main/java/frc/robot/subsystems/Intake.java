package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Intake extends SubsystemBase {
    private TalonFX motor;

    public Intake() {
        motor = new TalonFX(60); //TODO: Update motor ID
    }

    public void intakeObject() {
        motor.set(ControlMode.PercentOutput, 0.5);
    }

    public void extakeObject() {
        motor.set(ControlMode.PercentOutput, -0.5);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

}
