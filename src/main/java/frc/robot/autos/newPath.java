package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// import com.pathplanner.lib.auto.PathPlannerTrajectory;

public class newPath extends SequentialCommandGroup {
        public newPath(Swerve s_Swerve) {
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics);

                // This will load the file "Example Path.path" and generate it with a max
                // velocity of 4 m/s and a max acceleration of 3 m/s^2
                // PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("New Path",
                // new PathConstraints(4, 3));

                // // This trajectory can then be passed to a path follower such as a
                // // PPSwerveControllerCommand

                // var thetaController = new ProfiledPIDController(
                // Constants.AutoConstants.kPThetaController, 0, 0,
                // Constants.AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // s_Swerve::getPose,
                // Constants.Swerve.swerveKinematics,
                // new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                // new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                // thetaController,
                // s_Swerve::setModuleStates,
                // s_Swerve);

                // addCommands(
                // new InstantCommand(() ->
                // s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                // swerveControllerCommand);
        }
}