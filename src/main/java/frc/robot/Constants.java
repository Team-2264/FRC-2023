package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.enums.AutoPosition;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class ObjectVision {
        public static final String NETWORK_TABLE_ADDRESS = "ObjectVision";
    }

    public static final class AprilTag {
        public static final double CAMERA_HEIGHT_METERS = .130175;
        public static final double TARGET_HEIGHT_METERS = .180975;
        public static final double CAMERA_PITCH_RADIANS = 0;
        public static final double GOAL_RANGE_METERS = 0;
    }

    public static final class Arm {

        public static final int COMPRESSOR_ID = 33;

        public static final double GEAR_RATIO_ELBOW = 12.0;
        public static final double GEAR_RATIO_WRIST = 71.0;

        public static final int ARM_REVERSE_CHANNEL = 2;
        public static final int ARM_FORWARD_CHANNEL = 3;

        public static final int CLAW_REVERSE_CHANNEL = 1;
        public static final int CLAW_FORWARD_CHANNEL = 0;

        public static final int LEFT_ARM_MOTOR_ID = 30;
        public static final int RIGHT_ARM_MOTOR_ID = 31;

        public static final int WRIST_MOTOR_ID = 32;

        public static final boolean LEFT_ARM_MOTOR_INVERTED = false;

        public static final int SHOULDER_LIMIT_SWITCH_ID = 9;

        public static final int ENCODER_UNITS_PER_ROTATION_ELBOW = 2048;
        public static final int ENCODER_UNITS_PER_ROTATION_WRIST = 7;

        public static final double ENCODER_UNITS_PER_DEGREE_ELBOW = 68.267;
        public static final double ENCODER_UNITS_PER_DEGREE_WRIST = 5.45;
    }

    public static final class Swerve {
        public static final int pigeonID = 15;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25.5);
        public static final double wheelBase = Units.inchesToMeters(30);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.5;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent
                                                           // output for
                                                           // CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4; // meters per second
        public static final double maxAngularVelocity = Math.PI * 1.5; // radians per second (rad/s)

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Global Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int canCoderID = 48;
            // public static final double angleOffset = 101.68;
            public static final double angleOffset = 203.55;
            public static final boolean angleInverted = false;
            public static final boolean driveInverted = false;

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 42;
            public static final int angleMotorID = 43;
            public static final int canCoderID = 49;
            // public static final double angleOffset = 350.45;
            public static final double angleOffset = 25.8398;
            public static final boolean angleInverted = false;
            public static final boolean driveInverted = true;

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 46;
            public static final int angleMotorID = 47;
            public static final int canCoderID = 51;
            // public static final double angleOffset = 302.34375;
            // public static final double angleOffset = 236.7;
            public static final double angleOffset = 149.238;
            public static final boolean angleInverted = false;
            public static final boolean driveInverted = true;

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 44;
            public static final int angleMotorID = 45;
            public static final int canCoderID = 50;
            // public static final double angleOffset = 241.70;
            // public static final double angleOffset = 210.9;
            public static final double angleOffset = 127.705;
            public static final boolean angleInverted = false;
            public static final boolean driveInverted = false;

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        // public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        // public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 2;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 3.2;

        public static final double AUTO_BALANCE_MAX_SPEED_Y = 0.5; // m/s
        public static final double AUTO_BALANCE_MAX_SPEED_X = 0.5; // m/s

        public static final double ARM_ERROR_TOLERANCE = 100.0;

        public static final HashMap<AutoPosition, String> Trajectories = new HashMap<AutoPosition, String>() {
            {
                put(AutoPosition.INNER_CONE, AutoPosition.INNER_CONE.toString());
                put(AutoPosition.INNER, AutoPosition.INNER.toString());
                put(AutoPosition.INNER_BALANCE, AutoPosition.INNER_BALANCE.toString());
                put(AutoPosition.INNER_CONE_BALANCE, AutoPosition.INNER_CONE_BALANCE.toString());

                put(AutoPosition.EDGE, AutoPosition.EDGE.toString());
                put(AutoPosition.EDGE_CONE, AutoPosition.EDGE_CONE.toString());
                put(AutoPosition.EDGE_BALANCE, AutoPosition.EDGE_BALANCE.toString());
                put(AutoPosition.EDGE_CONE_BALANCE, AutoPosition.EDGE_CONE_BALANCE.toString());

                put(AutoPosition.CENTER, AutoPosition.CENTER.toString());
            }
        };
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

}
