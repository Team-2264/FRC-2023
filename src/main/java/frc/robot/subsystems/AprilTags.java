package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
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
        if (tagVision.hasTarget()) {
            
            SmartDashboard.putNumber("D", tagVision.getDistance());
            SmartDashboard.putString("D&T", tagVision.getDistanceAndTranslation().toString());
            
            tagVision.test_z_angle();
        }

    }

    public Pose2d getTargetPose(Swerve s_Swerve) {
        return tagVision.testPoseTransform(s_Swerve.getPose());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
