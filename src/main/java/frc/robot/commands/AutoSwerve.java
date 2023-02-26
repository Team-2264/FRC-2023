package frc.robot.commands;

import frc.robot.autos.PathGroupAuto;
import frc.robot.enums.AutoPosition;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutonomousEvents;
import frc.robot.Constants;
import frc.robot.Limelight;

public class AutoSwerve {

    private Swerve s_Swerve;
    private HashMap<String, Command> EVENT_MAP;
    private Limelight s_Limelight;
    private boolean balance;

    /**
     * @param s_Swerve    - Swerve Drive Subsystem
     * @param s_Arm       - subsystem for the robot Arm
     * @param s_Limelight - subsystem for the Limelight/Vision, expects a method
     *                    called "getAutonomousPosition" that returns an enum value
     *                    of the Robot's position
     */
    public AutoSwerve(Swerve s_Swerve, Arm s_Arm, Limelight s_Limelight, boolean balance) {
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        this.balance = balance;

        // initialize the event map which requires the Arm and Swerve Subsystems
        this.EVENT_MAP = new AutonomousEvents(s_Swerve, s_Arm).getEventMap();
    }

    /**
     * Selects the correct command to run based on the AprilTag being viewed by
     * limelight. Returns an Instant Command that resolves instantly if no AprilTag
     * is in vision.
     * 
     * @return {@link Command} to be scheduled and run by the Autonomous Scheduler
     */
    public Command getCommand() {

        // String EDGE_TRAJECTORY = (balance) ?
        // Constants.AutoConstants.EDGE_TRAJECTORY_BALANCE
        // : Constants.AutoConstants.EDGE_TRAJECTORY;
        // String INNER_BORDER_TRAJECTORY = (balance) ?
        // Constants.AutoConstants.INNER_BORDER_TRAJECTORY_BALANCE
        // : Constants.AutoConstants.INNER_BORDER_TRAJECTORY;

        // if (s_Limelight.getAutoPosition() == AutoPosition.EDGE)
        // return new PathGroupAuto(s_Swerve, EDGE_TRAJECTORY, EVENT_MAP);
        // if (s_Limelight.getAutoPosition() == AutoPosition.CENTER)
        // return new PathGroupAuto(s_Swerve, Constants.AutoConstants.CENTER_TRAJECTORY,
        // EVENT_MAP);
        // if (s_Limelight.getAutoPosition() == AutoPosition.INNER_BORDER)
        // return new PathGroupAuto(s_Swerve, INNER_BORDER_TRAJECTORY, EVENT_MAP);

        // // return new InstantCommand(() -> {
        // // });
        return new PathGroupAuto(s_Swerve, "New Path Copy", EVENT_MAP);

    }

}
