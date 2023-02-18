package frc.robot.commands;

import frc.robot.autos.PathPlannerAuto;
import frc.robot.autos.PathPlannerAutoWEvents;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class AutoSwerve {

    private Swerve s_Swerve;
    private Arm s_Arm;

    public AutoSwerve(Swerve s_Swerve, Arm s_Arm) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
    }

    double currentTime;

    public Command getCommand() {

        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        currentTime = System.currentTimeMillis();

        eventMap.put("raise_arm",
                new FunctionalCommand(this::doNothing, s_Arm::intake, this::doNothing,
                        () -> System.currentTimeMillis() - currentTime > 1, s_Arm));

        eventMap.put("drop_cargo",
                new FunctionalCommand(this::doNothing, s_Arm::toggleClaw, this::doNothing, () -> true, s_Arm));

        eventMap.put("retract_arm",
                new FunctionalCommand(this::doNothing, s_Arm::bringArmHome, this::doNothing, () -> true, s_Arm));

        // return new PathPlannerAutoWEvents(s_Swerve, "Test", eventMap);

        return new PathPlannerAuto(s_Swerve, "Cargo Autonomous");

    }

    public void doNothing(boolean ok) {

    }

    public void doNothing() {

    }

}
