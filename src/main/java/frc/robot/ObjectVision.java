package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectVision {
    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("ObjectVision");

    public ObjectVision() {};

    public double getYaw() {
        double yaw = TABLE.getValue("yaw").getDouble(); 
      
        return -yaw * (180/Math.PI);

    };

    public double getPitch() {
        double pitch = TABLE.getValue("pitch").getDouble(); 
      
        return pitch;

    };

    public boolean getObject() {
        return TABLE.getValue("detected").getBoolean();
    }

}
