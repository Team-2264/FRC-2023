package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ATVision;

public class AprilTags extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    ATVision tagVision;
    Pose3d currentATPose;

    public AprilTags() {
        tagVision = new ATVision();
        tagVision.init();
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    // public CommandBase exampleMethodCommand() {
    // // Inline construction of command goes here.
    // // Subsystem::RunOnce implicitly requires `this` subsystem.
    // return runOnce(
    // () -> {
    // /* one-time action goes here */
    // });
    // }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */

    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(tagVision.atcamera.getLatestResult().hasTargets()){
        currentATPose = tagVision.getRobotPose();
        SmartDashboard.putNumber("AprilTag Pose X", currentATPose.getX());
        SmartDashboard.putNumber("AprilTag Pose Y", currentATPose.getY());
        SmartDashboard.putNumber("AprilTag Pose Z", currentATPose.getZ());
        SmartDashboard.putNumber("AprilTag Pose Angle", currentATPose.getRotation().getAngle());
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
