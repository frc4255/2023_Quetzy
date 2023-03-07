package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class RobotState {
    
    private Spark rightLEDs;
    private Spark leftLEDs;

    public enum robotStates {
        CUBE, //101
        CONE, //102
        HAS_CUBE, //104
        HAS_CONE, //105
        NOT_BALANCED, //201
        BALANCED, //202
        ENCODER_DISCONNECTED, //401
        CAN_BUS_ERROR, //402
        IDLE, //103
        PROGRAMMERS_R_DUMB //403
    }
    

    private robotStates currentState = robotStates.IDLE;

    public RobotState() {
        rightLEDs = new Spark(1);
        leftLEDs = new Spark(0);

        rightLEDs.set(-0.99);
        leftLEDs.set(-0.99);
    }

    public void toggleState(int desiredState) {
        switch (desiredState) {
            case 101:
                currentState = robotStates.CUBE;
                rightLEDs.set(0.91);
                leftLEDs.set(0.91);
                break;
            case 102:
                currentState = robotStates.CONE;
                rightLEDs.set(0.69);
                leftLEDs.set(0.69);
                break;
            case 103:
                currentState = robotStates.IDLE;
                rightLEDs.set(-0.99);
                leftLEDs.set(-0.99);
                break;
            case 104:
                currentState = robotStates.HAS_CUBE;
                //rightLEDs.set();
                //leftLEDs.set();
                break;
            case 105:
                currentState = robotStates.HAS_CONE;
                //rightLEDs.set();
                //leftLEDs.set();
                break;
            case 201:
                currentState = robotStates.NOT_BALANCED;
                rightLEDs.set(-0.21);
                leftLEDs.set(-0.21);
                break;
            case 202:
                currentState = robotStates.BALANCED;
                rightLEDs.set(0.93);
                leftLEDs.set(0.93);
                break;
            case 401:
                currentState = robotStates.ENCODER_DISCONNECTED;
                rightLEDs.set(-0.11);
                leftLEDs.set(-0.11);
                break;
            case 402:
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
}
