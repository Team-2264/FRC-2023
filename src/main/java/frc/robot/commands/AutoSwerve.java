package frc.robot.commands;

import frc.robot.autos.PathPlannerAuto;
import frc.robot.autos.PathPlannerAutoWEvents;
import frc.robot.commands.ArmCommands.ArmOut;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class AutoSwerve {

    private Swerve s_Swerve;
    private Arm s_Arm;
    private ArmOut armOutCmd;

    public AutoSwerve(Swerve s_Swerve, Arm s_Arm) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
        this.armOutCmd = new ArmOut(s_Arm);
    }

    double currentTime;

    public Command getCommand() {

        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        eventMap.put("print", armOutCmd);
        // eventMap.put("print", new Print(
        // "PASSED MARK 1 THIS IS LITERALLY HHYPE I KNOW U CAN SEE THIS NEVE IF YOU CANT
        // ITS ACTUALLY OVER BRO"));

        // eventMap.put("drop_cargo",
        // new FunctionalCommand(this::doNothing, s_Arm::toggleClaw, this::doNothing, ()
        // -> true, s_Arm));

        // eventMap.put("retract_arm",
        // new FunctionalCommand(this::doNothing, s_Arm::bringArmHome, this::doNothing,
        // () -> true, s_Arm));

        return new PathPlannerAutoWEvents(s_Swerve, "f", eventMap);

        // return new PathPlannerAuto(s_Swerve, "Cargo Autonomous");

    }

    public void doNothing(boolean ok) {

    }

    public void doNothing() {

    }

}
