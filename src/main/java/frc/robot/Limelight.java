package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.enums.AutoPosition;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Limelight {

    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform2d currentTransform;
    double turnAngle;
    Translation2d distanceAndTranslation;
    ArrayList<ArrayList<Double>> positions;

    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public Limelight() {
        positions = new ArrayList<>();

        m_chooser.setDefaultOption("Middle", "Center");
        m_chooser.addOption("Outer", "Edge");
        m_chooser.addOption("Inner", "Inner");
        SmartDashboard.putData("Backup Autonomous", m_chooser);
    }

    double[] arrNull = { 0, 0, 0, 0, 0, 0 };
    double[] arr;

    public Pose3d getTargetToRobot() {
        double[] arr = TABLE.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        if (arr == arrNull) {
            return null;
        }

        return new Pose3d(arr[2], -arr[0], arr[1], new Rotation3d());
    }

    public Pose3d getBotPose() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            arr = TABLE.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }

        if (DriverStation.getAlliance() == Alliance.Red) {
            arr = TABLE.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        }

        if (arr == arrNull) {
            return null;
        }

        return new Pose3d(arr[0], arr[1], arr[2],
                new Rotation3d(arr[4] * Math.PI / 180, arr[3] * Math.PI / 180, arr[5] * Math.PI / 180));
    }

    double[] arrTwo;

    public Translation3d getBotTranslation() {

        if (DriverStation.getAlliance() == Alliance.Blue) {
            arrTwo = TABLE.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }

        if (DriverStation.getAlliance() == Alliance.Red) {
            arrTwo = TABLE.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        }

        if (arrTwo == arrNull) {
            return null;
        }

        return new Translation3d(arrTwo[0], arrTwo[1], arrTwo[2]);
    }

    double[] arrThree;

    public Double getBotAngle() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            arrThree = TABLE.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }
        if (DriverStation.getAlliance() == Alliance.Red) {
            arrThree = TABLE.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        }

        if (arrThree == arrNull)
            return null;

        return (arr[5] * Math.PI) / 180;
    }

    /*
     * public Pose3d getTargetToCamera () {
     * double[] arr =
     * NetworkTableInstance.getDefault().getTable("limelight").getEntry(
     * "targetpose_cameraspace").getDoubleArray(new double[6]);
     * 
     * SmartDashboard.putNumberArray("DTargetToCamera", arr);
     * return new Pose3d(arr[2], -arr[0], arr[1], new Rotation3d());
     * //return new Pose3d(arr[0], -arr[2], arr[1], new Rotation3d(arr[3], arr[4],
     * arr[5]));
     * }
     */

    // public void addNewPosition() {
    // positions.add(null)
    // }

    public Transform2d getTargetToCameraTransform() {
        Pose3d targetToCameraPose = getTargetToRobot();
        // SmartDashboard.putString("TargetToCamera", targetToCameraPose.toString());
        return new Transform2d(new Pose2d(), targetToCameraPose.toPose2d());
    }

    public int getTargetID() {
        return TABLE.getEntry("tid").getNumber(9).intValue();
    }

    public AutoPosition getAutoPosition() {
        int id = getTargetID();

        if (id == 1 || id == 8)
            return AutoPosition.EDGE;
        if (id == 2 || id == 7)
            return AutoPosition.CENTER;
        if (id == 3 || id == 6)
            return AutoPosition.INNER_BORDER;

        if(m_chooser.getSelected() == "Inner") return AutoPosition.INNER_BORDER;
        if(m_chooser.getSelected() == "Edge") return AutoPosition.EDGE;
        if(m_chooser.getSelected() == "Center") return AutoPosition.CENTER;

        return null;

    }
}
