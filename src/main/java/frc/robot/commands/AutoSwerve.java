package frc.robot.commands;

import frc.robot.autos.PathGroupAuto;
import frc.robot.enums.AutoPosition;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.AutonomousEvents;
import frc.lib.AutonomousPositions;
import frc.robot.Constants;

public class AutoSwerve {

    private Swerve s_Swerve;
    private HashMap<String, Command> EVENT_MAP;
    private HashMap<AutoPosition, String> POSITION_MAP;

    private AutoPosition position;

    /**
     * @param s_Swerve - Swerve Drive Subsystem
     * @param s_Arm    - Subsystem for the Robot Arm
     * @param position - Position of Robot
     */
    public AutoSwerve(Swerve s_Swerve, Arm s_Arm, AutoPosition position) {
        this.s_Swerve = s_Swerve;
        this.position = position;
        // initialize the event map which requires the Arm and Swerve Subsystems
        this.EVENT_MAP = new AutonomousEvents(s_Swerve, s_Arm).getEventMap();
        this.POSITION_MAP = new AutonomousPositions().getPositionMap();
    }

    /**
     * Selects the correct command to run based on the AprilTag being viewed by
     * limelight. Returns an Instant Command that resolves instantly if no AprilTag
     * is in vision.
     * 
     * @return {@link Command} to be scheduled and run by the Autonomous Scheduler
     */
    public Command getCommand() {

        if (this.position == AutoPosition.NONE || POSITION_MAP.get(position) == null)
            return new InstantCommand();

        return new PathGroupAuto(s_Swerve, POSITION_MAP.get(position),
                EVENT_MAP);

    }

}
