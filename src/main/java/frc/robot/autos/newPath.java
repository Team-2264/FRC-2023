package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class newPath extends SequentialCommandGroup {
        public newPath(Swerve s_Swerve) {

                // This will load the file "Example Path.path" and generate it with a max
                // velocity of 4 m/s and a max acceleration of 3 m/s^2
                PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("Diamond",
                                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                addCommands(
                                new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> System.out.println(s_Swerve.swerveOdometry.getPoseMeters())));
        }
}