package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class StopMoving extends CommandBase {

    private Swerve s_Swerve;

    public StopMoving(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
    }

    @Override
    public void execute() {
        s_Swerve.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
