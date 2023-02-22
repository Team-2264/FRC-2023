// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.enums.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick arm = new Joystick(1);

  private final Field2d m_field = new Field2d();

  private final int translationAxis = (int) (Joystick.AxisType.kY.value);
  private final int strafeAxis = (int) (Joystick.AxisType.kX.value);
  private final int rotationAxis = (int) (Joystick.AxisType.kZ.value);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kL3.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kR3.value);

  private final JoystickButton followTargetButton = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  private final JoystickButton followIntakeButton = new JoystickButton(driver, PS4Controller.Button.kSquare.value);

  private final JoystickButton clawToggleButton = new JoystickButton(arm, 1);
  private final JoystickButton armIntake = new JoystickButton(arm, 2);

  private final JoystickButton armSimbaCone = new JoystickButton(arm, 7);
  private final JoystickButton armSimbaCube = new JoystickButton(arm, 8);

  private final JoystickButton armMidCone = new JoystickButton(arm, 9);
  private final JoystickButton armMidCube = new JoystickButton(arm, 10);

  private final JoystickButton armLow = new JoystickButton(arm, 11);
  private final JoystickButton armLowIntake = new JoystickButton(arm, 12);

  // Emergency Buttons
  private final JoystickButton disableCommandButton = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);

  private static final NetworkTableInstance TABLE = NetworkTableInstance.getDefault();

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm s_Arm = new Arm();
  private final Limelight limelight = new Limelight();

  private TeleopAuto autoCommand;
  private TeleopAutoTwo autoCommandTwo;

  /**
   * `
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    BooleanSupplier fieldRelative = () -> robotCentric.getAsBoolean();
    boolean openLoop = true;

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve, driver, arm, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    SmartDashboard.putData("Field", m_field);
    // Configure the button bindings
    configureButtonBindings();

    // make sure we're not accidentally ready across reboots
    TABLE.getTable(Constants.ObjectVision.NETWORK_TABLE_ADDRESS).getEntry("ready").clearPersistent();

    // lets the listener on the raspberry pi know it's ready to process and send
    TABLE.getTable(Constants.ObjectVision.NETWORK_TABLE_ADDRESS).getEntry("ready").setBoolean(true);
  }

  public void resetEncoders() {
    s_Swerve.resetEncoders();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    followTargetButton.onTrue(new InstantCommand(() -> {
      if (limelight.getBotPose() != null) {
        s_Swerve.setPose(limelight.getBotPose().toPose2d());
        autoCommandTwo = new TeleopAutoTwo(
            s_Swerve,
            new Pose2d(
                s_Swerve.getPose().getX() - limelight.getTargetToRobot().getX() + 1,
                s_Swerve.getPose().getY() - limelight.getTargetToRobot().getY(),
                new Rotation2d(limelight.getBotAngle())),
            m_field);

        autoCommandTwo.schedule();
      }
    }));

    followIntakeButton.onTrue(new InstantCommand(() -> {
      if (limelight.getBotPose() != null) {
        s_Swerve.setPose(limelight.getBotPose().toPose2d());
        s_Arm.intake();
        autoCommandTwo = new TeleopAutoTwo(
            s_Swerve,
            new Pose2d(
                s_Swerve.getPose().getX() - limelight.getTargetToRobot().getX() + .7,
                s_Swerve.getPose().getY() - limelight.getTargetToRobot().getY() - .7,
                new Rotation2d(limelight.getBotAngle())),
            m_field);

        autoCommandTwo.schedule();
      }
    }));

    disableCommandButton.onTrue(new InstantCommand(() -> {
      autoCommand.end(true);
      autoCommandTwo.end(true);
    }));

    clawToggleButton.onTrue(new InstantCommand(() -> s_Arm.toggleClaw()));

    armIntake.onTrue(new InstantCommand(() -> {
      if (s_Arm.getStatus() == ArmStatus.HOME) {
        s_Arm.intake();
      } else {
        s_Arm.bringArmHome();
      }
    }));

    armLow.onTrue(new InstantCommand(() -> s_Arm.setLow()));
    armLowIntake.onTrue(new InstantCommand(() -> s_Arm.setLowIntake()));

    armMidCone.onTrue(new InstantCommand(() -> s_Arm.setMidCone()));
    armMidCube.onTrue(new InstantCommand(() -> s_Arm.setMidCube()));

    armSimbaCone.onTrue(new InstantCommand(() -> s_Arm.simbaCone()));
    armSimbaCube.onTrue(new InstantCommand(() -> s_Arm.simbaCube()));
  }

  /**
   * 
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command will run in autonomous
    return new AutoSwerve(s_Swerve, s_Arm, limelight).getCommand();
  }

  public void updateRobotPose() {
    m_field.setRobotPose(s_Swerve.getPose());
    // if(Math.abs(arm.getRawAxis(1)) > .1) s_Arm.adjustWrist(arm.getRawAxis(1));

    
  }

  public void armsInit() {
    s_Arm.init();
  }
}
