package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private String networkTableAddress;
    NetworkTable networkTable;

    public Vision(String networktablesAddress) {
        this.networkTableAddress = networktablesAddress;

        networkTable = NetworkTableInstance.getDefault().getTable(networkTableAddress);
    }

    public Translation2d getDistanceToTarget() {
        return new Translation2d();
    }

}
