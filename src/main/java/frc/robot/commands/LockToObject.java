package frc.robot.commands;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ObjectVision;
import frc.robot.subsystems.Swerve;

public class LockToObject extends CommandBase {

    private Swerve s_Swerve;
    private ObjectVision objectVision;

    double yaw;
    double pitch;
    double targetAngle;

    public LockToObject(Swerve s_Swerve, ObjectVision objectVision) {
        this.s_Swerve = s_Swerve;
        this.objectVision = objectVision;

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        yaw = objectVision.getYaw();
        pitch = objectVision.getPitch();
        this.targetAngle = s_Swerve.pidgey.getYaw() + yaw;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("FINAL OBJECT YAW", yaw);
        SmartDashboard.putNumber("TARGET OFFSET", s_Swerve.pidgey.getYaw() - targetAngle);

        yaw = objectVision.getYaw();

        s_Swerve.drive(new Translation2d(), (yaw / 90) * Constants.Swerve.maxAngularVelocity, false, false);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Swerve.pidgey.getYaw() - targetAngle) < 3;
    }
}
