// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final Field2d m_field = new Field2d();

  private final int translationAxis = (int) (Joystick.AxisType.kY.value);
  private final int strafeAxis = (int) (Joystick.AxisType.kX.value);
  private final int rotationAxis = (int) (Joystick.AxisType.kZ.value);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kL3.value); // val = 11
  private final JoystickButton followTargetButton = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  private final JoystickButton wristButton = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);

  private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL1.value);

  private POVButton straightForward = new POVButton(driver, 0);


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  private final Arm s_Arm = new Arm();

  private final ATVision at_Vision = new ATVision();

  // private final AprilTags s_AprilTags = new AprilTags();
 private teleopAuto autoCommand;

  /**
   * `
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    BooleanSupplier fieldRelative = () -> robotCentric.getAsBoolean();
    boolean openLoop = true;

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));


    SmartDashboard.putData("Field", m_field);
    // Configure the button bindings
    configureButtonBindings();
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
      s_Swerve.setPose(at_Vision.getBotPose().toPose2d());

      autoCommand = new teleopAuto(
        s_Swerve, 
        new Pose2d(
          s_Swerve.getPose().getX() + at_Vision.getTargetToRobot().getX() - 0.75, 
          s_Swerve.getPose().getY() + at_Vision.getTargetToRobot().getY(),
          new Rotation2d(at_Vision.getBotAngle())
        ), m_field
      );
      
      autoCommand.schedule();
    }));

    wristButton.onTrue(new InstantCommand(() -> s_Arm.toggleShoulder()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command will run in autonomous
    return new AutoSwerve(s_Swerve).getCommand();
  }

  public void updateRobotPose() {
    m_field.setRobotPose(s_Swerve.getPose());
  }
}
