package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class newPath extends SequentialCommandGroup {

        Swerve s_Swerve;
        PIDController thetaController;

        public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
                PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory, s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                                s_Swerve::setModuleStates, s_Swerve);
                return command;
        }

        public newPath(Swerve swerve) {
                this.s_Swerve = swerve;

                // This will load the file "Example Path.path" and generate it with a max
                // velocity of 4 m/s and a max acceleration of 3 m/s^2
                PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("New New Path",
                                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                addRequirements(s_Swerve);

                thetaController = new PIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0);

                // Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
                PPSwerveControllerCommand swerveControllerCommand2 = baseSwerveCommand(exampleTrajectory);

                addCommands(
                                new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                                swerveControllerCommand2,
                                new InstantCommand(() -> System.out.println(s_Swerve.swerveOdometry.getPoseMeters())));
        }
}