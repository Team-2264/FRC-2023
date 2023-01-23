package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlannerAuto extends SequentialCommandGroup {

        Swerve s_Swerve;
        PIDController thetaController;
        PathPlannerTrajectory trajectory;
        private PPSwerveControllerCommand command;

        public PathPlannerAuto(Swerve swerve, String pathName) {
                this.s_Swerve = swerve;
                addRequirements(s_Swerve);

                // loads the path with the constaints that we have set in the constants file
                this.trajectory = PathPlanner.loadPath(pathName,
                                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                thetaController = new PIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0);

                // Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);
                this.command = baseSwerveCommand(trajectory);

                this.setCommands();
        }

        public void setCommands() {
                addCommands(
                                new InstantCommand(() -> s_Swerve
                                                .resetOdometry(trajectory.getInitialHolonomicPose())),
                                command,
                                new InstantCommand(() -> System.out.println(s_Swerve.swerveOdometry.getPoseMeters())));
        }

        // returns a Path Planer Swerve Controller Command so we can use holonomic
        // driving
        public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
                PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory, s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics, // kinematics constants
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0), // PID for the X
                                                                                                // controller
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0), // PID for the Y
                                                                                                // controller
                                thetaController, // PID for the theta (angle)/rotation controller
                                s_Swerve::setModuleStates, // module state consumer
                                true, // if the path should be mirrored based on alliance color
                                s_Swerve // this uses the swerve subsystem
                );

                return command; // returns the command to be consumed by a follower
        }
}