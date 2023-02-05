package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.TurretConstants;

public class Limelight {
    NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTx() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0);
    }

    public double getTy() {
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(0);
    }

    public boolean hasTarget() {
        NetworkTableEntry tv = table.getEntry("tv");
        return (tv.getDouble(0) == 1) ? true : false;
    }

    public void setPipeline(int pipelineNo) {
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        pipeline.setInteger(pipelineNo);
    }

    public void setPipeline(TurretConstants.Apriltags selectedTag) {
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        switch (selectedTag) {
            case REDRIGHT1:
                pipeline.setInteger(0);
                break;
        
            case REDMIDDLE2:
                pipeline.setInteger(1);
                break;
            
            case REDLEFT3:
                pipeline.setInteger(2);
                break;

            case BLUERIGHT6:
                pipeline.setInteger(3);
                break;

            case BLUEMIDDLE7:
                pipeline.setInteger(4);
                break;

            case BLUELEFT8:
                pipeline.setInteger(5);
                break;

            case BLUEDOUBLE4:
                pipeline.setInteger(6);
                break;

            case REDDOUBLE5:
                pipeline.setInteger(7);
                break;
        }
    }
}
