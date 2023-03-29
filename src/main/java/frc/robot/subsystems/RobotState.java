package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class RobotState {
    
    private final Spark rightLEDs = new Spark(1);
    private final Spark leftLEDs = new Spark(0);

    public enum State {
        CUBE(0.91, 0.91),
        CONE(0.69, 0.69),
        HAS_OBJECT(-0.91, -0.91),
        NOT_BALANCED(-0.91, -0.91),
        BALANCED(0.93, 0.93),
        ENCODER_DISCONNECTED(-0.11, -0.11),
        CAN_BUS_ERROR(-0.09, -0.09),
        IDLE(-0.99, -0.99),
        PROGRAMMERS_R_DUMB(-0.55, -0.55);

        private final double rightValue;
        private final double leftValue;

        State(double rightValue, double leftValue) {
            this.rightValue = rightValue;
            this.leftValue = leftValue;
        }

        public double getRightLEDValue() {
            return rightValue;
        }

        public double getLeftLEDValue() {
            return leftValue;
        }
    }
    
    private State currentState = State.IDLE;
    private State lastState = State.IDLE;

    public RobotState() {
        rightLEDs.set(-0.99);
        leftLEDs.set(-0.99);
    }

    public void setState(State desiredState) {

        if (desiredState != currentState) {
            lastState = currentState;
        }
        currentState = desiredState;
        rightLEDs.set(currentState.getRightLEDValue());
        leftLEDs.set(currentState.getLeftLEDValue());
    }

    public State getCurrentState() {
        return currentState;
    }

    public State getLastState() {
        return lastState;
    }
}
