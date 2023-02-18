package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;

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
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private int rotationSetpoint;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis,
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

        if (this.controller.getPOV() == 0 || this.controller.getPOV() == 180) {
            rotationSetpoint = this.controller.getPOV();
        }

        if (rotationSetpoint != -1) {
            if (Math.abs((s_Swerve.gyro.getYaw() % 360) - rotationSetpoint) > 5) {
                rAxis = 0.3;
            } else {
                rotationSetpoint = -1;
            }
        }

        translation = new Translation2d(curve(yAxis), curve(xAxis)).times(Constants.Swerve.maxSpeed);
        rotation = rotationCurve(rAxis) * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, !fieldRelative.getAsBoolean(), openLoop);

    }

    public double curve(double input) {
        return (0.5 * input) + (0.2 * Math.pow(input, 3)) + (0.25 * (Math.pow(input, 5)));
    }

    public double rotationCurve(double input) {
        return (0.5 * input) + (0.25 * Math.pow(input, 3)) + (0.1 * (Math.pow(input, 5)));
    }

}
