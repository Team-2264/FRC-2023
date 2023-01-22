package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.Print;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlannerAutoWithEvents extends SequentialCommandGroup {

    Swerve s_Swerve;
    PIDController thetaController;

    public PathPlannerAutoWithEvents(Swerve swerve, String pathName, HashMap<String, Command> eventMap) {
        this.s_Swerve = swerve;
        addRequirements(s_Swerve);

        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("New New Path",
                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        thetaController = new PIDController(
                Constants.AutoConstants.kPThetaController, 0, 0);

        // Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        FollowPathWithEvents command = getCommand(trajectory, eventMap);

        addCommands(
                new InstantCommand(() -> s_Swerve
                        .resetOdometry(trajectory.getInitialHolonomicPose())),
                command);
    }

    public FollowPathWithEvents getCommand(PathPlannerTrajectory trajectory, HashMap<String, Command> eventMap) {
        return new FollowPathWithEvents(baseSwerveCommand(trajectory),
                trajectory.getMarkers(),
                eventMap);
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