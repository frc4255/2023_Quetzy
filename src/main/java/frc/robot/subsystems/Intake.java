package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.RobotState.State;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends SubsystemBase {
    private TalonFX motor;
    private RobotState s_RobotState;
    
    public Intake(RobotState s_RobotState) {
        motor = new TalonFX(30);
        motor.setNeutralMode(NeutralMode.Brake);

        this.s_RobotState = s_RobotState;
    }

    public void intakeObject() {
        motor.set(ControlMode.PercentOutput, 0.7);
    }

    public void extakeObject() {
        motor.set(ControlMode.PercentOutput, -0.7);
    }

    public void stop() {
        if (s_RobotState.getCurrentState() == State.CONE) {
            motor.set(ControlMode.PercentOutput, -0.07);
        } else if (s_RobotState.getCurrentState() == State.CUBE) {
            motor.set(ControlMode.PercentOutput, 0.07);
        } else {
            motor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public boolean hasObject() {
        if (motor.getStatorCurrent() > 50) {
            return true;
        } else {
            return false;
        }
    }

    public boolean extookObject() {
        if (motor.getStatorCurrent() > 26 && motor.getStatorCurrent() < 30) {
            return true;
        } else {
            return false;
        }
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor current", motor.getStatorCurrent());
    }
}
