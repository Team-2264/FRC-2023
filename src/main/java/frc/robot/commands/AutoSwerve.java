package frc.robot.commands;

import frc.robot.autos.PathPlannerAuto;
import frc.robot.autos.PathPlannerAutoWEvents;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSwerve {

    private Swerve s_Swerve;

    public AutoSwerve(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
    }

    public Command getCommand() {

        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        eventMap.put("marker1", new Print("Passed Marker 1"));
        return new PathPlannerAutoWEvents(s_Swerve, "Live", eventMap);

        // return new PathPlannerAuto(s_Swerve, "Live");

    }
}
