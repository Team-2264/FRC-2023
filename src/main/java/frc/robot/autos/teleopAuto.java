package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class teleopAuto extends SequentialCommandGroup {

    private SwerveControllerCommand swerveControllerCommand;
    private Pose2d currentpose;
    Trajectory exampleTrajectory;

    public teleopAuto(Swerve s_Swerve, Pose2d targetPose, Field2d field) {

        System.out.println("this is the target pose x" + targetPose.getX());

        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        currentpose = s_Swerve.getPose();

        // An example trajectory to follow. All units in meters.
        if (currentpose.getY() > 6.75 || (currentpose.getY() > 5.5 && currentpose.getX() < 3.3)
                || (currentpose.getY() > 5.5 && currentpose.getX() > 13.2)) {
            // DEADZONES
        } else if (DriverStation.getAlliance() == Alliance.Blue) {
            if (currentpose.getY() > 2.8) {
                if (currentpose.getX() < 3)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                            List.of(s_Swerve.getPose(), new Pose2d(2, 4.4, new Rotation2d(Math.PI))), config);
                else if (currentpose.getX() < 5)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(s_Swerve.getPose(),
                            new Pose2d(3.9, 4.7, new Rotation2d(Math.PI)), new Pose2d(2, 4.4, new Rotation2d(Math.PI))),
                            config);
                else
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(s_Swerve.getPose(),
                            new Pose2d(5.9, 4.7, new Rotation2d(Math.PI)),
                            new Pose2d(3.9, 4.7, new Rotation2d(Math.PI)), new Pose2d(2, 4.4, new Rotation2d(Math.PI))),
                            config);
                setDashboard("BLUE TOP");
            } else {
                if (currentpose.getX() < 3)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                            List.of(s_Swerve.getPose(), new Pose2d(2, 1, new Rotation2d(Math.PI))), config);
                else if (currentpose.getX() < 5)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(s_Swerve.getPose(),
                            new Pose2d(3.9, .9, new Rotation2d(Math.PI)), new Pose2d(2, 1, new Rotation2d(Math.PI))),
                            config);
                else
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(s_Swerve.getPose(),
                            new Pose2d(5.9, .9, new Rotation2d(Math.PI)), new Pose2d(3.9, .9, new Rotation2d(Math.PI)),
                            new Pose2d(2, 1, new Rotation2d(Math.PI))), config);
                setDashboard("BLUE BOTTOM");
            }
        } else {
            if (currentpose.getY() > 2.8) {
                if (currentpose.getX() < 3)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                            List.of(s_Swerve.getPose(), new Pose2d(14.6, 4.4, new Rotation2d(0))), config);
                else if (currentpose.getX() < 5)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(s_Swerve.getPose(),
                            new Pose2d(12.6, 4.7, new Rotation2d(0)), new Pose2d(14.6, 4.4, new Rotation2d(0))),
                            config);
                else
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                            List.of(s_Swerve.getPose(), new Pose2d(5.9, 4.7, new Rotation2d(0)),
                                    new Pose2d(12.6, 4.7, new Rotation2d(0)), new Pose2d(14.6, 4.4, new Rotation2d(0))),
                            config);
                setDashboard("RED TOP");
            } else {
                if (currentpose.getX() > 13.6)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                            List.of(s_Swerve.getPose(), new Pose2d(14.6, 1, new Rotation2d(0))), config);
                else if (currentpose.getX() > 11.5)
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(s_Swerve.getPose(),
                            new Pose2d(12.6, .9, new Rotation2d(0)), new Pose2d(14.6, 1, new Rotation2d(Math.PI))),
                            config);
                else
                    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                            List.of(s_Swerve.getPose(), new Pose2d(10.6, .9, new Rotation2d(0)),
                                    new Pose2d(12.6, .9, new Rotation2d(0)), new Pose2d(14.6, 1, new Rotation2d(0))),
                            config);
                setDashboard("RED BOTTOM");
            }
        }

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

    private void setDashboard(String input) {
        // SmartDashboard.putString("CURRENT TELEAUTO", input);
        // SmartDashboard.putString("CURRENTPOSE RAW", currentpose.getX() + " " +
        // currentpose.getY());
    }

    public void stop() {
        swerveControllerCommand.end(true);
    }
}