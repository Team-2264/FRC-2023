package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PathPlannerAutoWEvents extends PathPlannerAuto {

    private FollowPathWithEvents command;

    public PathPlannerAutoWEvents(Swerve swerve, String pathName, HashMap<String, Command> eventMap) {
        super(swerve, pathName);

        // creates a new PathPlanner FollowPathWithEvents command so we can actually use
        // the events. Pass in the eventMap so we can use the events as Commands
        this.command = getCommand(trajectory, eventMap);
    }

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
