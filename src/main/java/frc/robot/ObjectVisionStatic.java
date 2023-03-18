package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectVisionStatic {
    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("ObjectVision");

    public static double getYaw() {
        double yaw = TABLE.getValue("yaw").getDouble();
        return yaw;
    };

    public static double getPitch() {
        double pitch = TABLE.getValue("pitch").getDouble();
        return pitch;
    };
}
