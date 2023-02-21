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

import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedList;

public class TeleopAuto extends SequentialCommandGroup {

    private SwerveControllerCommand swerveControllerCommand;
    private Pose2d currentpose;
    private Swerve s_Swerve;
    Trajectory exampleTrajectory;

    Pose2d blueTop, blueBottom, redTop, redBottom;

    Deque<Pose2d> waypoints;

    TrajectoryConfig config;

    public TeleopAuto(Swerve inputSwerve, Field2d field) {

        s_Swerve = inputSwerve;

        waypoints = new LinkedList<>();

        blueTop = new Pose2d(2.3, 4.4, new Rotation2d(Math.PI));
        blueBottom = new Pose2d(2.3, 1, new Rotation2d(Math.PI));
        
        redTop = new Pose2d(14.3, 4.4, new Rotation2d(0));
        redBottom = new Pose2d(14.3, 1, new Rotation2d(0));
        
        config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        currentpose = s_Swerve.getPose();

        // An example trajectory to follow. All units in meters.
        if (currentpose.getY() > 6.75 || (currentpose.getY() > 5.5 && currentpose.getX() < 3.3)
                || (currentpose.getY() > 5.5 && currentpose.getX() > 13.2)) {
            // DEADZONES NEED TO FIND A WAY TO END CODE
        } else if (DriverStation.getAlliance() == Alliance.Blue) {
            if (currentpose.getY() > 2.8) {
                waypoints = createWaypoints(blueTop, "blue", "top"); 
            } else {
                waypoints = createWaypoints(blueBottom, "blue", "bottom"); 
            }

            actuallyDoCommand(field);
        } else {
            if (currentpose.getY() > 2.8) {
                waypoints = createWaypoints(redTop, "red", "top"); 
            } else {
                waypoints = createWaypoints(redBottom, "red", "bottom"); 
            }

            actuallyDoCommand(field);
        }

    }

    private void actuallyDoCommand(Field2d field) {
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(new ArrayList<Pose2d>(waypoints), config);

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

    private Pose2d createWaypoint(Pose2d finalWaypoint, double tX, double tY) {
        return new Pose2d(finalWaypoint.getX() + tX, finalWaypoint.getY() + tY, finalWaypoint.getRotation());
    }

    private Deque<Pose2d> createWaypoints(Pose2d finalWaypoint, String alliance, String side) {
        Deque<Pose2d> output = new LinkedList<>();

        output.addFirst(finalWaypoint);
        if (currentpose.getX() > 3 && currentpose.getX() < 13.6)
            output.addFirst(createWaypoint(finalWaypoint, (alliance == "blue" ? 1: -1) * 1.6, side == "top" ? .3 : -.1));
        if (currentpose.getX() > 5 && currentpose.getX() < 11.6)
            output.addFirst(createWaypoint(finalWaypoint, (alliance == "blue" ? 1: -1) * 3.6, side == "top" ? .3 : -.1));
        return output;
    }

    public void stop() {
        swerveControllerCommand.end(true);
    }
}