package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.MovementDirection;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoBalance extends CommandBase {

    private Swerve s_Swerve;
    private MovementDirection direction;
    private boolean balanced = false;
    private long lastBalanced;
    private double DEGREE_DEADZONE = 7.5;

    public AutoBalance(Swerve s_Swerve, MovementDirection direction) {
        this.s_Swerve = s_Swerve;
        this.direction = direction;
        this.lastBalanced = -1;
    }

    @Override
    public void initialize() {

    }

    double currAngle;

    @Override
    public void execute() {
        SmartDashboard.putNumber("ANGLE WE THINK", currAngle);
        if (!balanced) {

            currAngle = s_Swerve.pidgey.getRoll();
            double speedX = Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X;

            speedX = (currAngle > 5) ? Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X
                    : Constants.AutoConstants.AUTO_BALANCE_MAX_SPEED_X / 3;

            if (currAngle > DEGREE_DEADZONE) {
                // lastBalanced = -1;
                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                new ChassisSpeeds(-speedX, 0, 0)));

                SmartDashboard.putString("DIRECTION", "FORWARD");

                // SmartDashboard.putNum
            } else if (currAngle < -DEGREE_DEADZONE) {
                // lastBalanced = -1;
                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                new ChassisSpeeds(speedX, 0, 0)));

                SmartDashboard.putString("DIRECTION", "REVERSE");
            } else {
                // if (lastBalanced == -1)
                // lastBalanced = System.currentTimeMillis();
                // else if (System.currentTimeMillis() - lastBalanced > 100)

                s_Swerve.setModuleStates(
                        Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,
                                0, 0)));

                balanced = true;

            }
        } else {

            SmartDashboard.putString("DIRECTION", "STOPPED");

            s_Swerve.setModuleStates(
                    Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,
                            0, 0)));

        }

    }

    public void end() {
        s_Swerve.setModuleStates(
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,
                        0, 0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
