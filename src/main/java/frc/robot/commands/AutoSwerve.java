package frc.robot.commands;

import frc.robot.autos.PathPlannerAuto;
import frc.robot.autos.PathPlannerAutoWEvents;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoSwerve {

    private Swerve s_Swerve;
    private Arm s_Arm;

    public AutoSwerve(Swerve s_Swerve, Arm s_Arm) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
    }

    public Command getCommand() {

        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        eventMap.put("raise_arm", new InstantCommand(() -> {
            s_Arm.intake();
        }));

        eventMap.put("drop_cargo", new InstantCommand(() -> {
            s_Arm.toggleClaw();
        }));

        eventMap.put("retract_arm", new InstantCommand(() -> {
            s_Arm.bringArmHome();
        }));

        return new PathPlannerAutoWEvents(s_Swerve, "Cargo Autonomous", eventMap);

        // return new PathPlannerAuto(s_Swerve, "Live");

    }
}
