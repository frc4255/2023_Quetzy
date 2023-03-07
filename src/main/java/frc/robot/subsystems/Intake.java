package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends SubsystemBase {
    private TalonFX motor;
    
    public Intake() {
        motor = new TalonFX(30);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void intakeObject() {
        motor.set(ControlMode.PercentOutput, 0.35);
    }

    public void extakeObject() {
        motor.set(ControlMode.PercentOutput, -0.35);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean hasObject() {
        if (motor.getStatorCurrent() > 60) {
            return true;
        } else {
            return false;
        }
    }
}
