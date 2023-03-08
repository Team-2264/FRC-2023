package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private BooleanSupplier fieldRelative;
    private boolean openLoop;

    private Swerve s_Swerve;
    private Joystick controller;
    private Joystick arm;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private double rotationSetpoint;
    private double startingRotation, finalRotation;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, Joystick arm, int translationAxis, int strafeAxis,
            int rotationAxis,
            BooleanSupplier fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.rotationSetpoint = -1;
        this.arm = arm;

    }

    public double curve(double input) {
        return (0.5 * input) + (0.2 * Math.pow(input, 3)) + (0.25 * (Math.pow(input, 5)));
    }

    public double rotationCurve(double input) {
        // return (0.5 * input) + (0.25 * Math.pow(input, 3)) + (0.3 * (Math.pow(input,
        // 5)));
        // return (Math.pow(input,3));
        return input;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(curve(yAxis), curve(xAxis)).times(Constants.Swerve.maxSpeed);
        rotation = rotationCurve(rAxis) * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, !fieldRelative.getAsBoolean(), openLoop);

    }

    public double normalizedGyro() {
        SmartDashboard.putNumber("NORMALIZED GYRO", Math.abs(s_Swerve.pidgey.getYaw() % 360));
        return Math.abs(s_Swerve.pidgey.getYaw() % 360);
    }

}
