package frc.robot.commands;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ObjectVision;
import frc.robot.autos.TeleopAutoTwo;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GoToObject extends CommandBase {

    private Swerve s_Swerve;
    private ObjectVision objectVision;

    double pitch;

    public GoToObject(Swerve s_Swerve, ObjectVision objectVision) {
        this.s_Swerve = s_Swerve;
        this.objectVision = objectVision;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        pitch = objectVision.getPitch();
    }

    @Override
    public void execute() {

        pitch = objectVision.getPitch();

        // new TeleopAutoTwo(s_Swerve, new Pose2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY(), new Rotation2d()), null);


    }

    @Override
    public boolean isFinished() {
        return pitch > .17;
    }
}
