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
    private double startingRotation;

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
        return (0.5 * input) + (0.25 * Math.pow(input, 3)) + (0.1 * (Math.pow(input, 5)));
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

        if (this.controller.getRawButton(7)) {
            rotationSetpoint = .3;
            startingRotation = s_Swerve.pidgey.getYaw();
        }

        if (this.controller.getRawButton(8)) {
            rotationSetpoint = -.3;
            startingRotation = s_Swerve.pidgey.getYaw();
        }

        if (rotationSetpoint != -1) {
            if (Math.abs(s_Swerve.pidgey.getYaw() % 180) > 3) {
                rAxis = rotationSetpoint;
                SmartDashboard.putString("spinning", "ifone");
            } else if (Math.abs(s_Swerve.pidgey.getYaw() - startingRotation) < 5) {
                rAxis = rotationSetpoint;
                SmartDashboard.putString("spinning", "ifone");
            } else {
                rotationSetpoint = -1;
                rAxis = 0;
                SmartDashboard.putString("spinning", "else");
            }
        }

        if (Math.abs(controller.getRawAxis(rotationAxis)) > Constants.stickDeadband) {
            rotationSetpoint = -1;
            rAxis = -controller.getRawAxis(rotationAxis);
        }

        if (Math.abs(arm.getRawAxis(2)) > .2) {
            rAxis = arm.getRawAxis(2) * -.2;
        }

        translation = new Translation2d(curve(yAxis), curve(xAxis)).times(Constants.Swerve.maxSpeed);
        rotation = rotationCurve(rAxis) * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, !fieldRelative.getAsBoolean(), openLoop);

    }

}
