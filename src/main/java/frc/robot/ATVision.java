package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.autos.teleopAuto;
import frc.robot.subsystems.Swerve;

import java.util.List;

import java.math.*;

public class ATVision {

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobot = new Transform3d(new Translation3d(Units.feetToMeters(1), 0.0, Units.feetToMeters(1)),
            new Rotation3d(0, 0, 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
                                      // from center.

    

    public ATVision() {

    }


    Transform2d currentTransform;
    double turnAngle;
    Translation2d distanceAndTranslation;

    public Pose3d getTargetToRobot () {
        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumberArray("DTargetToCamera", arr);
        return new Pose3d(arr[2], -arr[0], arr[1], new Rotation3d());
        //return new Pose3d(arr[0], -arr[2], arr[1], new Rotation3d(arr[3], arr[4], arr[5]));
    }

    /*public Pose3d getTargetToCamera () {
        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumberArray("DTargetToCamera", arr);
        return new Pose3d(arr[2], -arr[0], arr[1], new Rotation3d());
        //return new Pose3d(arr[0], -arr[2], arr[1], new Rotation3d(arr[3], arr[4], arr[5]));
    }*/

    
    // UNTESTED (also does not take into consideration the offset between the robot and the camera)
    public Pose2d getRobotFieldPosition() {
        double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        long tag_id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);

        Pose3d targetToCameraPose = new Pose3d(arr[0], arr[1], arr[2], new Rotation3d());
        SmartDashboard.putString("TargetToCamera", targetToCameraPose.toString());
        Pose3d tagPose = aprilTagFieldLayout.getTagPose((int) tag_id).get();

        return new Pose3d().relativeTo(tagPose).toPose2d();
        
    }

    public Transform2d getTargetToCameraTransform() {
        Pose3d targetToCameraPose = getTargetToRobot();
        SmartDashboard.putString("TargetToCamera", targetToCameraPose.toString());
        return new Transform2d(new Pose2d(), targetToCameraPose.toPose2d());
    }
}
