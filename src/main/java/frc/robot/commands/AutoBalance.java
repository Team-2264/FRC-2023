package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.MovementDirection;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {

    private Swerve s_Swerve;
    private MovementDirection direction;
    private boolean balanced = false;

    public AutoBalance(Swerve s_Swerve, MovementDirection direction) {
        this.s_Swerve = s_Swerve;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        switch (direction) {
            case FORWARD:
                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                new ChassisSpeeds(Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X, 0, 0)));
                break;
            case BACKWARD:
                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                new ChassisSpeeds(-Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X, 0, 0)));
                break;
            case LEFT:
                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                new ChassisSpeeds(0, -Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_Y, 0)));
                break;
            case RIGHT:
                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(
                                0, Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_Y, 0)));
                break;
            case RELATIVE:
                s_Swerve.setModuleStates(Constants.Swerve.swerveKinematics
                        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                                Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X, 0.0, 0.0, s_Swerve.getYaw())));
            default:
                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
                break;
        }
    }

    @Override
    public void execute() {
        double currAngle = s_Swerve.pidgey.getPitch();

        if (currAngle > 2.5) {
            s_Swerve.setModuleStates(
                    Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                            new ChassisSpeeds(Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X / 2, 0, 0)));
        } else if (currAngle < -2.5) {
            s_Swerve.setModuleStates(
                    Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                            new ChassisSpeeds(-Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X / 2, 0, 0)));
        } else {
            s_Swerve.setModuleStates(
                    Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
            balanced = true;
        }
    }

    @Override
    public boolean isFinished() {
        return balanced;
    }

}
