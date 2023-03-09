package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class RobotState {
    
    private Spark rightLEDs;
    private Spark leftLEDs;

    public enum robotStates {
        CUBE, //101
        CONE, //102
        HAS_OBJECT, //104
        NOT_BALANCED, //201
        BALANCED, //202
        ENCODER_DISCONNECTED, //401
        CAN_BUS_ERROR, //402
        IDLE, //103
        PROGRAMMERS_R_DUMB //403
    }
    

    private robotStates currentState = robotStates.IDLE;
    private robotStates lastState = robotStates.IDLE;
    public RobotState() {
        rightLEDs = new Spark(1);
        leftLEDs = new Spark(0);

        rightLEDs.set(-0.99);
        leftLEDs.set(-0.99);
    }

    public void toggleState(int desiredState) {
        
        switch (desiredState) {
            case 101:
                if (robotStates.CUBE != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.CUBE;
                rightLEDs.set(0.91);
                leftLEDs.set(0.91);
                break;
            case 102:
                if (robotStates.CONE != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.CONE;
                rightLEDs.set(0.69);
                leftLEDs.set(0.69);
                break;
            case 103:
                if (robotStates.IDLE != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.IDLE;
                rightLEDs.set(-0.99);
                leftLEDs.set(-0.99);
                break;
            case 104:
                if (robotStates.HAS_OBJECT != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.HAS_OBJECT;
                rightLEDs.set(-0.91); //TODO: Setup REV Blinkin to have green as color 1 or 2
                leftLEDs.set(-0.91);
                break;
            case 201:
                if (robotStates.NOT_BALANCED != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.NOT_BALANCED;
                rightLEDs.set(-0.21);
                leftLEDs.set(-0.21);
                break;
            case 202:
                if (robotStates.BALANCED != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.BALANCED;
                rightLEDs.set(0.93);
                leftLEDs.set(0.93);
                break;
            case 401:
                if (robotStates.ENCODER_DISCONNECTED != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.ENCODER_DISCONNECTED;
                rightLEDs.set(-0.11);
                leftLEDs.set(-0.11);
                break;
            case 402:
                if (robotStates.CAN_BUS_ERROR != currentState) {
                    lastState = currentState;
                }
                currentState = robotStates.CAN_BUS_ERROR;
                rightLEDs.set(-0.09);
                leftLEDs.set(-0.09);
                break;
            default:
                currentState = robotStates.PROGRAMMERS_R_DUMB;
                rightLEDs.set(-0.55);
                leftLEDs.set(-0.55);
                break;
        }
    }

    public robotStates getCurrentState() {
        return currentState;
    }

    public robotStates getLastState() {
        return lastState;
    }
}
