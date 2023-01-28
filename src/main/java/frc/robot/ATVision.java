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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.*;

import frc.robot.Constants;
import frc.robot.autos.teleopAuto;
import frc.robot.subsystems.Swerve;

import java.util.List;

import java.math.*;

public class ATVision {

    public PhotonCamera atcamera = new PhotonCamera("ApriTag Camera"); // Update UI with this info

    PhotonPipelineResult result;
    Pose2d test2d = new Pose2d();
    List<PhotonTrackedTarget> targets;
    PhotonTrackedTarget bestTarget;
    boolean hasTargets;
    boolean camAdded;
    double yaw;
    double pitch;
    double area;
    Transform2d pose;
    List<TargetCorner> corners;
    int targetID;
    double poseAmbiguity;
    Transform3d bestCameraToTarget;
    Transform3d alternateCameraToTarget;
    Transform3d cameraToTarget;
    Path ATPath = Paths.get("../src/main/java/frc/robot/ATFieldLayout.java");
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobot = new Transform3d(new Translation3d(Units.feetToMeters(1), 0.0, Units.feetToMeters(1)),
            new Rotation3d(0, 0, 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
                                      // from center.
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    Optional<Pose3d> optionalFieldRelativeTagPose;
    Pose3d fieldRelativeTagPose;

    public ATVision() {

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2022-rapidreact.json");
            SmartDashboard.putString("DO ATFL BE MADE?", "YES");
        } catch (Exception e) {
            SmartDashboard.putString("DO ATFL BE MADE?", "NO LMAO");
        }

        if (!camAdded) {
            camList.add(new Pair<PhotonCamera, Transform3d>(atcamera, cameraToRobot));
            camAdded = true;

        }
    }

    // public void updateCamera(){
    // result = atcamera.getLatestResult();
    // hasTargets = result.hasTargets();
    // if(!hasTargets){
    // return;
    // }
    // bestTarget = result.getBestTarget();
    // yaw = bestTarget.getYaw();
    // pitch = bestTarget.getPitch();
    // area = bestTarget.getArea();
    // corners = bestTarget.getDetectedCorners();
    // targetID = bestTarget.getFiducialId();
    // bestCameraToTarget = bestTarget.getBestCameraToTarget();
    // alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();
    // poseAmbiguity = bestTarget.getPoseAmbiguity();
    // }

    // public double getAngle() {
    // result = atcamera.getLatestResult();
    // return result;
    // }

    public boolean hasTarget() {
        result = atcamera.getLatestResult();
        return result.hasTargets();

    }

    public double getDistance() {
        result = atcamera.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.AprilTag.CAMERA_HEIGHT_METERS,
                    Constants.AprilTag.TARGET_HEIGHT_METERS,
                    Constants.AprilTag.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return -1;
    }

    double d;

    public Translation2d getDistanceAndTranslation() {
        result = atcamera.getLatestResult();

        d = getDistance();

        return PhotonUtils.estimateCameraToTargetTranslation(d,
                Rotation2d.fromDegrees(-(result.getBestTarget().getYaw())));

    }

    // returns your robot’s Pose3d on the field using the pose of the AprilTag
    // relative to the camera, pose of the AprilTag relative to the field, and the
    // transform from the camera to the origin of the robot.
    public Pose3d getRobotPose() {
        result = atcamera.getLatestResult();
        if (result.hasTargets()) {
            bestTarget = result.getBestTarget();
            cameraToTarget = bestTarget.getAlternateCameraToTarget();
            optionalFieldRelativeTagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
            if (optionalFieldRelativeTagPose.isPresent()) {
                fieldRelativeTagPose = optionalFieldRelativeTagPose.get();
                return PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget,
                        fieldRelativeTagPose, cameraToRobot);

            }

        }
        return null;

    }

    Transform2d currentTransform;
    double turnAngle;
    Translation2d distanceAndTranslation;

    public Pose2d testPoseTransform(Pose2d currentPose) {
        distanceAndTranslation = getDistanceAndTranslation();

        turnAngle = Math.atan(distanceAndTranslation.getY() / distanceAndTranslation.getX());

        SmartDashboard.putString("hi", distanceAndTranslation.toString());

        return currentPose.plus(new Transform2d(getDistanceAndTranslation(), new Rotation2d())); // NOT WORKING PLS DONT
                                                                                                 // USE

    }
}
