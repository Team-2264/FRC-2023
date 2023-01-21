// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

  /* Drive Controls */
  // private final int translationAxis = PS4Controller.Axis.kLeftY.value;
  // private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  // private final int rotationAxis = PS4Controller.Axis.kRightX.value;

  private final int translationAxis = (int) (Joystick.AxisType.kY.value);
  private final int strafeAxis = (int) (Joystick.AxisType.kX.value);
  private final int rotationAxis = (int) (Joystick.AxisType.kZ.value);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kL3.value); // val = 11
  private final JoystickButton resetEncoder = new JoystickButton(driver, 8);
  private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL1.value);

  private POVButton straightForward = new POVButton(driver, 0);

  // private final JoystickButton openClawButton = new JoystickButton(driver, 10);
  // private final JoystickButton closeClawButton = new JoystickButton(driver, 9);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  private final AprilTags s_AprilTags = new AprilTags();

  // private final AprilTags s_AprilTags = new AprilTags();

  /**
   * `
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    BooleanSupplier fieldRelative = () -> robotCentric.getAsBoolean();
    boolean openLoop = true;

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  public void resetEncoders() {
    s_Swerve.resetEncoders();

  }

  public void runForward() {
    if (driver.getPOV() == 0) {
      s_Swerve.driveStraight();
    }
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
    // resetEncoder.onTrue(new InstantCommand(() -> s_Swerve.resetEncoders()));
    // openClawButton.onTrue(new InstantCommand(() -> s_Arm.openClaw()));
    // closeClawButton.whenPressed(new InstantCommand(() -> s_Arm.closeClaw()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
