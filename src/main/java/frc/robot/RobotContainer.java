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

  /* Drive Controls */
  // private final int translationAxis = PS4Controller.Axis.kLeftY.value;
  // private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
  // private final int rotationAxis = PS4Controller.Axis.kRightX.value;

  private final int translationAxis = (int) (Joystick.AxisType.kY.value);
  private final int strafeAxis = (int) (Joystick.AxisType.kX.value);
  private final int rotationAxis = (int) (Joystick.AxisType.kZ.value);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kL3.value); // val = 11
  private final JoystickButton followTargetButton = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  private final JoystickButton solenoidTrigger = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
  private final JoystickButton compressorTrigger = new JoystickButton(driver, PS4Controller.Button.kSquare.value);


  private final JoystickButton resetEncoder = new JoystickButton(driver, 8);
  private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL1.value);

  private POVButton straightForward = new POVButton(driver, 0);

  // private final JoystickButton openClawButton = new JoystickButton(driver, 10);
  // private final JoystickButton closeClawButton = new JoystickButton(driver, 9);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  private final AprilTags s_AprilTags = new AprilTags();

  // private final Arm s_Arm = new Arm();

  private final ATVision at_Vision = new ATVision();

  // private final AprilTags s_AprilTags = new AprilTags();
 private teleopAuto autoCommand;

 private Pose2d[] last_april_poses = new Pose2d[30];
 private int next_april_pose_index = 0;

 

  /**
   * `
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    BooleanSupplier fieldRelative = () -> robotCentric.getAsBoolean();
    boolean openLoop = true;

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));


        SmartDashboard.putData("Field", m_field);
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

  public void testAprilTag() {
    Transform2d transform = at_Vision.getTargetToCameraTransform();
  
    Pose2d tPose = new Pose2d(1, 0, Rotation2d.fromDegrees(180)).transformBy(transform);

    SmartDashboard.putString("tPose", tPose.toString());
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

      // SmartDashboard.putString("bPose", at_Vision.getBotPose().toString());

      // SmartDashboard.putString("swerve pose", s_Swerve.getPose().toString());

      // SmartDashboard.putString("translation", at_Vision.getTargetToRobot().getTranslation().toString());

      // SmartDashboard.putString("target pose", new Pose2d(s_Swerve.getPose().getX() + at_Vision.getTargetToRobot().getX(), 
      // s_Swerve.getPose().getY() + at_Vision.getTargetToRobot().getY(),
      // new Rotation2d()).toString());


      s_Swerve.setPose(at_Vision.getBotPose().toPose2d());

      autoCommand = new teleopAuto(s_Swerve, new Pose2d(s_Swerve.getPose().getX() + at_Vision.getTargetToRobot().getX() - 0.75, 
      s_Swerve.getPose().getY() + at_Vision.getTargetToRobot().getY(),
      new Rotation2d(at_Vision.getBotAngle())), m_field);
      
      autoCommand.schedule();
      
    }));
     
    // solenoidTrigger.onTrue(new InstantCommand(() -> s_Arm.toggleLowerArm()));
    //compressorTrigger.onTrue(new InstantCommand(() -> s_Arm.toggleCompressor()));
    
    /*
    followTargetButton.onFalse(new InstantCommand(() -> {
      if (autoCommand != null) {
        autoCommand.stop();
      }
    }));*/
    // openClawButton.onTrue(new InstantCommand(() -> s_Arm.openClaw()));
    // closeClawButton.whenPressed(new InstantCommand(() -> s_Arm.closeClaw()));
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
  // // s_Swerve.setPose(at_Vision.getBotPose().toPose2d());
  // // last_april_poses[next_april_pose_index] = at_Vision.getBotPose().toPose2d();
  // // next_april_pose_index += 1;
  // // if(next_april_pose_index == 30) {
  // //   next_april_pose_index = 0;
  // // }


  // Translation2d translation = new Translation2d();
  // Rotation2d rotation = new Rotation2d();

  // for (Pose2d pose : last_april_poses) {
  //   var x = translation.getX() + pose.getX();
  //   var y = translation.getY() + pose.getY();
  // }

  
  m_field.setRobotPose(s_Swerve.getPose());
}


}
