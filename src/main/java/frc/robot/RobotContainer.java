// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.AutonomousEvents;
import frc.lib.AutonomousPositions;
import frc.robot.GamepieceDetection.ObjectDetection;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.HashMap;
import java.util.Optional;

import frc.robot.enums.*;


import frc.robot.commands.AutoCloseClaw;

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

  private final int translationAxis = (int) (Joystick.AxisType.kY.value);
  private final int strafeAxis = (int) (Joystick.AxisType.kX.value);
  private final int rotationAxis = (int) (Joystick.AxisType.kZ.value);

  /* Driver Buttons */
  private final JoystickButton lockToObject = new JoystickButton(driver, PS4Controller.Button.kR2.value);

  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kL3.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kR3.value);

  private final JoystickButton clawToggleButton = new JoystickButton(arm, 1);
  private final JoystickButton armIntake = new JoystickButton(arm, 2);

  private final JoystickButton armSimbaCone = new JoystickButton(arm, 7);
  private final JoystickButton armSimbaCube = new JoystickButton(arm, 8);

  private final JoystickButton armMidCone = new JoystickButton(arm, 9);
  private final JoystickButton armMidCube = new JoystickButton(arm, 10);

  private final JoystickButton autoIntake = new JoystickButton(arm, 11);
  private final JoystickButton armLowIntake = new JoystickButton(arm, 12);

  private final JoystickButton resetWrist = new JoystickButton(arm, 5);

  private final JoystickButton moveWristDown = new JoystickButton(arm, 3);
  private final JoystickButton moveWristUp = new JoystickButton(arm, 4);

  private final JoystickButton forceReset = new JoystickButton(arm, 6);
  // Emergency Buttons
  private final JoystickButton disableCommandButton = new JoystickButton(driver, PS4Controller.Button.kL1.value);

  // private static final NetworkTableInstance TABLE =
  // NetworkTableInstance.getDefault();

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm s_Arm = new Arm(arm);
  private final ObjectVision objectVision = new ObjectVision();

  private TeleopAuto autoCommand;
  private TeleopAutoTwo autoCommandTwo;

  SendableChooser<String> autoPathChooser = new SendableChooser<>();

  private LockToObject lockObject;

  private final AutoSwerve autoSwerve = new AutoSwerve(s_Swerve, s_Arm);

  /**
   * `
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    BooleanSupplier fieldRelative = () -> robotCentric.getAsBoolean();
    boolean openLoop = true;

    lockObject = new LockToObject(s_Swerve, objectVision);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve, driver, arm, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    // Configure the button bindings
    configureButtonBindings();

    // make sure we're not accidentally ready across reboots
    // TABLE.getTable(Constants.ObjectVision.NETWORK_TABLE_ADDRESS).getEntry("ready").clearPersistent();

    // // lets the listener on the raspberry pi know it's ready to process and send
    // TABLE.getTable(Constants.ObjectVision.NETWORK_TABLE_ADDRESS).getEntry("ready").setBoolean(true);

    for (AutoPosition position : AutoPosition.values()) {
      if (position == AutoPosition.INNER_CONE)
        autoPathChooser.setDefaultOption(position.toString(), position.toString());
      else
        autoPathChooser.addOption(position.toString(), position.toString());
    }

    SmartDashboard.putData("Auto Pos.", autoPathChooser);

    PowerDistribution pdh = new PowerDistribution();

    pdh.clearStickyFaults();

    pdh.close();

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

    try {
      if (objectVision.getObject()) {
        lockToObject.whileTrue(lockObject);
      }
    } catch (Exception e) {
      System.out.println(e);
    }

    // testingButton.onTrue(new InstantCommand(() -> s_Arm.forceEnableArm()));
    autoIntake.onTrue(new AutoCloseClaw(s_Arm, s_Swerve));

    /* Driver Buttons */

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    disableCommandButton.onTrue(new InstantCommand(() -> {
      if (autoCommand.isScheduled())
        autoCommand.end(true);
      if (autoCommandTwo.isScheduled())
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

    // armLow.onTrue(new InstantCommand(() -> s_Arm.midIntake()));
    forceReset.onTrue(new InstantCommand(() -> s_Arm.forceEnableArm()));
    armLowIntake.onTrue(new InstantCommand(() -> s_Arm.setLowIntake()));

    armMidCone.onTrue(new InstantCommand(() -> s_Arm.setMidCone()));
    armMidCube.onTrue(new InstantCommand(() -> s_Arm.setMidCube()));

    armSimbaCone.onTrue(new InstantCommand(() -> s_Arm.simbaCone()));
    armSimbaCube.onTrue(new InstantCommand(() -> s_Arm.simbaCube()));

    resetWrist.onTrue(new InstantCommand(() -> s_Arm.init()));

    moveWristDown.onTrue(new InstantCommand(() -> s_Arm.wristPosition += 20));
    moveWristUp.onTrue(new InstantCommand(() -> s_Arm.wristPosition -= 20));
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSwerve.getCommand(getAutoPosition());

    // Command will run in autonomous
    // return new AutoSwerve(s_Swerve, s_Arm, getAutoPosition(), EVENT_MAP,
    // POSITION_MAP)
    // .getCommand();
  }

  public AutoPosition getAutoPosition() {
    String failSafeString = autoPathChooser.getSelected();

    // this is the default return value
    AutoPosition failSafe = AutoPosition.NONE;

    for (AutoPosition position : AutoPosition.values()) {
      if (failSafeString == position.toString())
        failSafe = position;
    }

    return failSafe;
  }

  public String getAutoTraj() {
    return autoPathChooser.getSelected();
  }

  public void setYawToCurrentPose() {
    s_Swerve.pidgey.setYaw(s_Swerve.getPose().getRotation().getDegrees());
  }

  public void postCurrentAutonomousCommand() {
    SmartDashboard.putString("Current Autonomous Command",
        getAutoPosition().toString());

  }

  public void armsInit() {
    s_Arm.init();
  }
}



// followMiddle.onTrue(new InstantCommand(() -> {
//   if (limelight.getAutoPosition() != null) {
//     s_Swerve.setPose(limelight.getBotPose().toPose2d());
//     autoCommandTwo = new TeleopAutoTwo(
//         s_Swerve,
//         new Pose2d(
//             s_Swerve.getPose().getX() - limelight.getTargetToRobot().getX() + .87,
//             s_Swerve.getPose().getY() - limelight.getTargetToRobot().getY(),
//             new Rotation2d((Math.PI))),
//         m_field);

//     autoCommandTwo.schedule();
//     if (driver.getRawButton(PS4Controller.Button.kR1.value))
//       s_Arm.simbaCube();
//     else
//       s_Arm.setMidCube();
//   }
// }));

// // followLeft.onTrue(new InstantCommand(() -> {
// // if (limelight.getAutoPosition() != null) {
// // s_Swerve.setPose(limelight.getBotPose().toPose2d());
// // autoCommandTwo = new TeleopAutoTwo(
// // s_Swerve,
// // new Pose2d(
// // s_Swerve.getPose().getX() - limelight.getTargetToRobot().getX() + .87,
// // s_Swerve.getPose().getY() - limelight.getTargetToRobot().getY() + .53,
// // new Rotation2d((DriverStation.getAlliance() == Alliance.Blue ? Math.PI :
// // Math.PI))),
// // m_field);

// // autoCommandTwo.schedule();
// // if (driver.getRawButton(PS4Controller.Button.kR1.value))
// // s_Arm.simbaCone();
// // else
// // s_Arm.setMidCone();
// // }
// // }));

// followIntakeButton.onTrue(new InstantCommand(() -> {
//   if ((int) limelight.getTargetID() != -1) {

//     s_Swerve.setPose(LimelightHelpers.getBotPose2d_wpiBlue(""));

//     // s_Arm.intake();
//     autoCommandTwo = new TeleopAutoTwo(
//         s_Swerve,
//         new Pose2d(
//             14.2,
//             7.3,
//             new Rotation2d(Math.PI / 2)),
//         m_field);

//     autoCommandTwo.schedule();
//   }
// }));

// followRight.onTrue(new InstantCommand(() -> {
//   if ((int) limelight.getTargetID() == 4 || (int) limelight.getTargetID() == 5) {

//     s_Swerve.setPose(LimelightHelpers.getBotPose2d_wpiBlue(""));

//     // s_Arm.intake();
//     if (DriverStation.getAlliance() == Alliance.Blue) {
//       autoCommandTwo = new TeleopAutoTwo(
//           s_Swerve,
//           new Pose2d(
//               15.3,
//               6,
//               new Rotation2d(0)),
//           m_field);
//     } else {

//       autoCommandTwo = new TeleopAutoTwo(
//           s_Swerve,
//           new Pose2d(
//               1.23,
//               7.5,
//               new Rotation2d(Math.PI)),
//           m_field);
//     }

//     autoCommandTwo.schedule();
//   }
// }));

// followLeft.onTrue(new InstantCommand(() -> {
//   if ((int) limelight.getTargetID() == 4 || (int) limelight.getTargetID() == 5) {

//     s_Swerve.setPose(LimelightHelpers.getBotPose2d_wpiBlue(""));

//     // s_Arm.intake();
//     if (DriverStation.getAlliance() == Alliance.Blue) {
//       autoCommandTwo = new TeleopAutoTwo(
//           s_Swerve,
//           new Pose2d(
//               15.3,
//               7.5,
//               new Rotation2d(0)),
//           m_field);
//     } else {
//       autoCommandTwo = new TeleopAutoTwo(
//           s_Swerve,
//           new Pose2d(
//               1.23,
//               6,
//               new Rotation2d(Math.PI)),
//           m_field);
//     }

//     autoCommandTwo.schedule();
//   }
// }));