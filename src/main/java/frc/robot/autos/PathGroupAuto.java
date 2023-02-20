package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Child of the PathPlannerAuto class.
 *
 * Allows Path Following along with triggered events along the route
 */
public class PathGroupAuto extends SequentialCommandGroup {

        private Swerve s_Swerve;

        /**
         * Creates a new PathGroupAuto object that will return a command to follow a
         * PathPlanner path along with PathPlannerLib's event system
         *
         * @param swerve
         * @param pathName
         * @param eventMap
         */
        public PathGroupAuto(Swerve s_Swerve, String pathName, HashMap<String, Command> eventMap) {
                this.s_Swerve = s_Swerve;

                // loads the path group with the constaints that we have set in the constants
                // file
                ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner
                                .loadPathGroup(
                                                pathName,
                                                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                // creates the auto builder
                SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(s_Swerve::getPose, this::resetOdometry,
                                Constants.Swerve.swerveKinematics,
                                new PIDConstants(Constants.AutoConstants.kPXController, 0, 0), new PIDConstants(
                                                Constants.AutoConstants.kPThetaController, 0, 0),
                                s_Swerve::setModuleStates, eventMap, s_Swerve);

                // creates the full auto command
                Command fullAuto = autoBuilder.fullAuto(pathGroup);

                // actually make the command, command, yk? ⏣

                addCommands(fullAuto);
        }

        private void resetOdometry(Pose2d pose) {
                s_Swerve.pidgey.setYaw(pose.getRotation().getDegrees());
                s_Swerve.resetOdometry(pose);
        }

}
