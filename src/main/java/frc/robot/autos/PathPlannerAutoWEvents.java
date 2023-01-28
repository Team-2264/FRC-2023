package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Child of the PathPlannerAuto class.
 * 
 * Allows Path Following along with triggered events along the route
 */
public class PathPlannerAutoWEvents extends PathPlannerAuto {

    private FollowPathWithEvents command;

    /**
     * Creates a new PathPlannerAuto object that will return a command to follow a
     * PathPlanner path along with PathPlannerLib's event system
     * 
     * @param swerve
     * @param pathName
     * @param eventMap
     */
    public PathPlannerAutoWEvents(Swerve swerve, String pathName, HashMap<String, Command> eventMap) {
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

        // creates a new PathPlanner FollowPathWithEvents command so we can actually use
        // the events. Pass in the eventMap so we can use the events as Commands
        this.command = getCommand(trajectory, eventMap);

        this.setCommands();

    }

    /**
     * 
     * 
     * @see frc.robot.autos.PathPlannerAuto#setCommands()
     */
    @Override
    public void setCommands() {
        // add those commands to the sequential command group
        addCommands(
                new InstantCommand(() -> s_Swerve
                        .resetOdometry(trajectory.getInitialHolonomicPose())),
                command);
    }

    // returns the FollowPathWithEvents command so we can use the events, really
    // just a nice abstraction
    public FollowPathWithEvents getCommand(PathPlannerTrajectory trajectory, HashMap<String, Command> eventMap) {
        return new FollowPathWithEvents(baseSwerveCommand(trajectory),
                trajectory.getMarkers(),
                eventMap);
    }
}
