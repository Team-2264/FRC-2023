package frc.robot.commands;

import frc.robot.autos.PathGroupAuto;
import frc.robot.enums.AutoPosition;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutonomousEvents;
import frc.robot.Limelight;

public class AutoSwerve {

    private Swerve s_Swerve;
    private HashMap<String, Command> EVENT_MAP;
    private Limelight s_Limelight;

    public AutoSwerve(Swerve s_Swerve, Arm s_Arm, Limelight s_Limelight) {
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        this.EVENT_MAP = new AutonomousEvents(s_Swerve, s_Arm).getEventMap();
    }

    double currentTime;

    public Command getCommand() {
        if (s_Limelight.getAutoPosition() == AutoPosition.EDGE)
            return new PathGroupAuto(s_Swerve, "Edge", EVENT_MAP);
        if (s_Limelight.getAutoPosition() == AutoPosition.CENTER)
            return new PathGroupAuto(s_Swerve, "Center", EVENT_MAP);
        if (s_Limelight.getAutoPosition() == AutoPosition.INNER_BORDER)
            return new PathGroupAuto(s_Swerve, "Inner", EVENT_MAP);

        return new InstantCommand(() -> {
        });
    }

}
