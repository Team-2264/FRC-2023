package frc.robot.autos;

import frc.robot.ATVision;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;
import java.util.function.BooleanSupplier;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class teleopAuto extends SequentialCommandGroup {

    private SwerveControllerCommand swerveControllerCommand;

    public teleopAuto(Swerve s_Swerve, Pose2d targetPose, Field2d field) {

        System.out.println("this is the target pose x" + targetPose.getX());

        TrajectoryConfig config = new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(s_Swerve.getPose(), targetPose), config);

        field.getObject("traj").setTrajectory(exampleTrajectory);


        var thetaController = new ProfiledPIDController(
                        Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        swerveControllerCommand = new SwerveControllerCommand(
                        exampleTrajectory,
                        s_Swerve::getPose,
                        Constants.Swerve.swerveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);

        addCommands(swerveControllerCommand);
        
    }

    
    public void stop() {
        swerveControllerCommand.end(true);
    }
}