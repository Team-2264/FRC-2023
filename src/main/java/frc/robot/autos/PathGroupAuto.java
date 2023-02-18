package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Child of the PathPlannerAuto class.
 *
 * Allows Path Following along with triggered events along the route
 */
public class PathGroupAuto extends SequentialCommandGroup {

        Swerve s_Swerve;
        PIDController thetaController;
        PathPlannerTrajectory trajectory;
        SwerveAutoBuilder autoBuilder;

        /**
         * Creates a new PathPlannerAuto object that will return a command to follow a
         * PathPlanner path along with PathPlannerLib's event system
         *
         * @param swerve
         * @param pathName
         * @param eventMap
         */
        public PathGroupAuto(Swerve swerve, String pathName, HashMap<String, Command> eventMap) {

                this.s_Swerve = swerve;

                // loads the path with the constaints that we have set in the constants file
                ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner
                                .loadPathGroup(
                                                pathName,
                                                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                autoBuilder = new SwerveAutoBuilder(s_Swerve::getPose, s_Swerve::resetOdometry,
                                Constants.Swerve.swerveKinematics,
                                new PIDConstants(Constants.AutoConstants.kPXController, 0, 0), new PIDConstants(
                                                Constants.AutoConstants.kPThetaController, 0, 0),
                                s_Swerve::setModuleStates, eventMap, s_Swerve);

                Command fullAuto = autoBuilder.fullAuto(pathGroup);

                addCommands(fullAuto);
        }

}
