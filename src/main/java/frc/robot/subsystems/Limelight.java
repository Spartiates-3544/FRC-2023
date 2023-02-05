package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
}
