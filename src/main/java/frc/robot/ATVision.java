package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class ATVision {

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobot = new Transform3d(new Translation3d(Units.feetToMeters(1), 0.0, Units.feetToMeters(1)),
            new Rotation3d(0, 0, 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
                                      // from center.

    ArrayList<ArrayList<Double>> positions;
    

    public ATVision() {
        positions = new ArrayList<>();
    }


    Transform2d currentTransform;
    double turnAngle;
    Translation2d distanceAndTranslation;

    public Pose3d getTargetToRobot () {
        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumberArray("DTargetToCamera", arr);
        return new Pose3d(arr[2], -arr[0], arr[1], new Rotation3d());
    }

    public Pose3d getBotPose() {

        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
        
        return new Pose3d (arr[0], arr[1], arr[2], new Rotation3d(arr[4]*Math.PI/180, arr[3]*Math.PI/180, arr[5]*Math.PI/180));
    }


    public Translation3d getBotTranslation() {

        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        SmartDashboard.putNumberArray("DBotPose", arr);
        
        return new Translation3d (arr[0], arr[1], arr[2]);
    }

    public double getBotAngle() {
        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        return (arr[4] * Math.PI)/180;
    }

    /*public Pose3d getTargetToCamera () {
        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumberArray("DTargetToCamera", arr);
        return new Pose3d(arr[2], -arr[0], arr[1], new Rotation3d());
        //return new Pose3d(arr[0], -arr[2], arr[1], new Rotation3d(arr[3], arr[4], arr[5]));
    }*/

    // public void addNewPosition() {
    //     positions.add(null)
    // }


    public Transform2d getTargetToCameraTransform() {
        Pose3d targetToCameraPose = getTargetToRobot();
        SmartDashboard.putString("TargetToCamera", targetToCameraPose.toString());
        return new Transform2d(new Pose2d(), targetToCameraPose.toPose2d());
    }
}
