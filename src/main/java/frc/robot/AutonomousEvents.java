package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.StopMoving;
import frc.robot.commands.arm.*;
import frc.robot.enums.ArmStatus;
import frc.robot.enums.MovementDirection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class AutonomousEvents {

    public static final HashMap<String, Command> EVENT_MAP = new HashMap<String, Command>();

    public AutonomousEvents(Swerve s_Swerve, Arm s_Arm) {
        EVENT_MAP.put("HOME", new SetArmToPosition(s_Arm, ArmStatus.HOME));

        EVENT_MAP.put("CONE_SIMBA", new SetArmToPosition(s_Arm, ArmStatus.CONE_SIMBA));
        EVENT_MAP.put("CUBE_SIMBA", new SetArmToPosition(s_Arm, ArmStatus.CUBE_SIMBA));

        EVENT_MAP.put("CUBE_MID", new SetArmToPosition(s_Arm, ArmStatus.CUBE_MID));
        EVENT_MAP.put("CONE_MID", new SetArmToPosition(s_Arm, ArmStatus.CONE_MID));

        EVENT_MAP.put("LOW", new SetArmToPosition(s_Arm, ArmStatus.LOW));

        EVENT_MAP.put("OPEN_CLAW", new OpenClaw(s_Arm));
        EVENT_MAP.put("CLOSE_CLAW", new CloseClaw(s_Arm));

        EVENT_MAP.put("BALANCE_FORWARD", new AutoBalance(s_Swerve, MovementDirection.FORWARD));
        EVENT_MAP.put("BALANCE_BACKWARD", new AutoBalance(s_Swerve, MovementDirection.BACKWARD));
        EVENT_MAP.put("BALANCE_RELATIVE", new AutoBalance(s_Swerve, MovementDirection.RELATIVE));

        EVENT_MAP.put("HALT", new StopMoving(s_Swerve));
    }

    public HashMap<String, Command> getEventMap() {
        return EVENT_MAP;
    }

}
