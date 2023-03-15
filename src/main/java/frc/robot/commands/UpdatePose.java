package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.enums.UpdateType;
import frc.robot.subsystems.Swerve;

public class UpdatePose extends CommandBase {
    private UpdateType updateType;
    private Swerve s_Swerve;

    public UpdatePose(Swerve s_Swerve, UpdateType updateType) {
        this.updateType = updateType;
        this.s_Swerve = s_Swerve;
    }

    @Override
    public void initialize() {
        execute();
    }

    @Override
    public void execute() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            s_Swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue(""));
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            s_Swerve.resetOdometry(LimelightHelpers.getBotPose2d_wpiRed(""));
        }

    }

    public boolean isFinished() {
        return updateType == UpdateType.ONCE;
    }

}
