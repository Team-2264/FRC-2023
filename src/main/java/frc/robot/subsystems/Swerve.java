package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 pidgey;

    public Swerve() {
        pidgey = new Pigeon2(Constants.Swerve.pigeonID);
        pidgey.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants,
                        Constants.Swerve.Mod0.angleInverted,
                        Constants.Swerve.Mod0.driveInverted),
                new SwerveModule(1, Constants.Swerve.Mod1.constants,
                        Constants.Swerve.Mod1.angleInverted,
                        Constants.Swerve.Mod1.driveInverted),
                new SwerveModule(2, Constants.Swerve.Mod2.constants,
                        Constants.Swerve.Mod2.angleInverted,
                        Constants.Swerve.Mod2.driveInverted),
                new SwerveModule(3, Constants.Swerve.Mod3.constants,
                        Constants.Swerve.Mod3.angleInverted,
                        Constants.Swerve.Mod3.driveInverted),

        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    /**
     * @Deprecated
     *             DANGEROUS!! Do NOT use this unless testing the robot. It is set
     *             to work with
     *             the absolute encoders, this method is only for if they are not
     *             working.
     */
    public void resetEncoders() {
        for (SwerveModule mod : mSwerveMods) {
            mSwerveMods[mod.moduleNumber].resetToAbsolute();
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

    }

    /**
     * Drives around a specified center of rotation. Use this for tracking around a
     * cone or cube
     * 
     * @param translation
     * @param rotation
     * @param fieldRelative
     * @param isOpenLoop
     * @param translationFromCenter A {@link Translation2d} of the distance from the
     *                              center of the robot you want to rotate from
     */
    public void driveAroundPoint(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            Translation2d translationFromCenter) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation),
                translationFromCenter);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void rotateAroundPoint(Translation2d distanceFromCenter) {
        driveAroundPoint(distanceFromCenter, 0.75, false, false, distanceFromCenter);
    }

    public void driveStraight() {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(new ChassisSpeeds(1.0, 0, 0));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void driveStraightBack() {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(new ChassisSpeeds(-1, 0, 0));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void stop() {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        pidgey.setYaw(0);
    }

    public void invertGyro() {
        pidgey.setYaw(180);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 -
                pidgey.getYaw())
                : Rotation2d.fromDegrees(pidgey.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());

        SmartDashboard.putNumber("Gyro", pidgey.getYaw());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
            // mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
            // mod.getState().speedMetersPerSecond);

        }
    }
}