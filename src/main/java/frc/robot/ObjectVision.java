package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectVision {
    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("ObjectVision");

    public ObjectVision() {
    };

    public double getYaw() {
        try {
            double yaw = TABLE.getValue("yaw").getDouble();
            return -yaw * (180 / Math.PI);
        } catch (Exception e) {
            return 0;
        }

    };

    public double getPitch() {
        try {
            double pitch = TABLE.getValue("pitch").getDouble();
            return pitch;
        } catch (Exception e) {
            return 0;
        }

    };

    public boolean getObject() {
        try {
            return TABLE.getValue("detected").getBoolean();
        } catch (Exception e) {
            return false;
        }
    }

}
