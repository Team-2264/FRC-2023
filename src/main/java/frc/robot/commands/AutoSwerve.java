package frc.robot.commands;

import frc.robot.autos.PathPlannerAutoWithEvents;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoSwerve {

    private Swerve s_Swerve;

    public AutoSwerve(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
    }

    public Command getCommand() {

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new Print("Passed Marker 1"));
        return new PathPlannerAutoWithEvents(s_Swerve, "Holonomic Forward", eventMap);

        // return new PathPlannerAuto(s_Swerve, "Holonomic Forward");

    }
}
