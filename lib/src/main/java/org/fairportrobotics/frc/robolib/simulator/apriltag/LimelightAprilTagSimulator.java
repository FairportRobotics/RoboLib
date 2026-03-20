package org.fairportrobotics.frc.robolib.simulator.apriltag;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class LimelightAprilTagSimulator {
    
    private String cameraName;
    private NetworkTableInstance ntInstance;
    private NetworkTable table;
    private TrackingType trackType;


    private int tv;
    private double tx;
    private double ty;


    public LimelightAprilTagSimulator(String limelightName, TrackingType trackingType){
        this.cameraName = limelightName;
        ntInstance = NetworkTableInstance.getDefault();
        table = ntInstance.getTable(limelightName);
        this.trackType = trackingType;
    }

    public void simulationUpdate(){

        


        // Publish new vals
        table.putValue("tv", NetworkTableValue.makeInteger(tv));
        table.putValue("tx", NetworkTableValue.makeDouble(tx));
        table.putValue("ty", NetworkTableValue.makeDouble(ty));
    }

    public enum TrackingType{
        BOT_POSE,
        STANDARD
    }

}
