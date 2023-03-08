package frc.lib.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    
    private int port;

    public Limelight(int port) {
        this.port = port;
    }

    public void LEDoff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void LEDon() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void LEDblink() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }

    public double getTv() {
        return NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("tv").getDouble(0);
    }

    public double getTx() {
        return NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("tx").getDouble(0);
    }

    public double getTy() {
        return NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("ty").getDouble(0);
    }

    public double getTa() {
        return NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("ta").getDouble(0);
    }

    public boolean hasTarget() {
        if (getTv() == 0) {
            return false;
        } else {
            return true;
        }
    }
    
}
