package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    
    private int port;

    private double tV;
    private double tX;
    private double tY;
    private double tA;
    
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
        return tV;
    }

    public double getTx() {
        return tX;
    }

    public double getTy() {
        return tY;
    }

    public double getTa() {
        return tA;
    }

    public boolean hasTarget() {
        if (getTv() == 0) {
            return false;
        } else {
            return true;
        }
    }

    public void update() {
        tV = NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("tv").getDouble(0);
        tX = NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("tx").getDouble(0);
        tY = NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("tu").getDouble(0);
        tA = NetworkTableInstance.getDefault().getTable("limelight" + Integer.toString(port)).getEntry("ta").getDouble(0);
    }   

    @Override
    public void periodic() {
        update();
    }
    
}

