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
import edu.wpi.first.apriltag.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.autos.teleopAuto;
import frc.robot.subsystems.Swerve;

import java.util.List;

import java.math.*;

public class ATVision {
    public PhotonCamera atcamera = new PhotonCamera("AprilTag Camera"); // Update UI with this info

    PhotonPipelineResult result;

    boolean hasTargets;
    boolean camAdded;

    Transform3d bestCameraToTarget;
    Transform3d alternateCameraToTarget;
    Transform3d cameraToTarget;

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

        } catch (Exception e) {

        }

        if (!camAdded) {
            camList.add(new Pair<PhotonCamera, Transform3d>(atcamera, cameraToRobot));
            camAdded = true;

        }

    }

    public void test_z_angle() {
        result = atcamera.getLatestResult();

        PhotonTrackedTarget target;
        target = result.getBestTarget();

        Double zAngle = target.getBestCameraToTarget().inverse().getRotation().getAngle();

        SmartDashboard.putNumber("Z Angle", zAngle);

    }

    public boolean hasTarget() {
        result = atcamera.getLatestResult();
        return result.hasTargets();

    }

    public double getDistance() {
        result = atcamera.getLatestResult();

        return PhotonUtils.calculateDistanceToTargetMeters(
                Constants.AprilTag.CAMERA_HEIGHT_METERS,
                Constants.AprilTag.TARGET_HEIGHT_METERS,
                Constants.AprilTag.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));

    }

    public Translation2d getDistanceAndTranslation() {
        result = atcamera.getLatestResult();

        double d = getDistance();

        return PhotonUtils.estimateCameraToTargetTranslation(d,
                Rotation2d.fromDegrees(-(result.getBestTarget().getYaw())));

    }

    Transform2d currentTransform;
    double turnAngle;
    Translation2d distanceAndTranslation;

    public Pose2d testPoseTransform(Pose2d currentPose) {
        distanceAndTranslation = getDistanceAndTranslation();

        turnAngle = Math.atan(distanceAndTranslation.getY() / distanceAndTranslation.getX());

        return currentPose.plus(new Transform2d(getDistanceAndTranslation(), new Rotation2d()));

    }

}
