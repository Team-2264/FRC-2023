package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
// NO
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pneumatics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {

  TalonFX leftArmBelt, rightArmBelt;
  Pneumatics leftArm, rightArm;
  Compressor compressor;
  boolean compressorOn = false;
  DigitalInput leftLimitSwitch, rightLimitSwitch;

  public Arm() {
    leftArm = new Pneumatics(Constants.Arm.LEFT_ARM_FORWARD_CHANNEL, Constants.Arm.LEFT_ARM_REVERSE_CHANNEL);
    rightArm = new Pneumatics(Constants.Arm.RIGHT_ARM_FORWARD_CHANNEL, Constants.Arm.RIGHT_ARM_REVERSE_CHANNEL);
    compressor = new Compressor(42, PneumaticsModuleType.REVPH);
    leftArmBelt = new TalonFX(Constants.Arm.LEFT_ARM_MOTOR_ID);
    rightArmBelt = new TalonFX(Constants.Arm.RIGHT_ARM_MOTOR_ID);
    leftLimitSwitch = new DigitalInput(Constants.Arm.LEFT_LIMIT_SWITCH_ID);
    rightLimitSwitch = new DigitalInput(Constants.Arm.RIGHT_LIMIT_SWITCH_ID);

    // compressor.enableAnalog(2, 120);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public CommandBase exampleMethodCommand() {
  // // Inline construction of command goes here.
  // // Subsystem::RunOnce implicitly requires `this` subsystem.
  // return runOnce(
  // () -> {
  // /* one-time action goes here */
  // });
  // }

  // public void closeClaw() {
  // leftArmServo.set(0.5);
  // rightArmServo.set(0.5);
  // System.out.println("Neev and his stupid ass claw");
  // }

  // public void openClaw() {
  // leftArmServo.set(0);
  // rightArmServo.set(0);
  // System.out.println("Neev and his stupid ass claw but open");
  // }

  // chandan complete the heart <3
  public void toggleLowerArm() {
    leftArm.solenoidToggle();
    rightArm.solenoidToggle();
  }

  public void moveUp(){
    if(!(leftArmBelt.getSelectedSensorPosition() * Constants.Arm.circumferenceOfGear == 3.0 )) {
      leftArmBelt.set(ControlMode.PercentOutput, 1);
      rightArmBelt.set(ControlMode.PercentOutput, 1);
    }
  }


  public void moveToPos(double angle) {
    double leftDesiredEncoderPos = angle/Constants.Arm.LEFT_ENCODER_UNIS_PER_ANGLE;
    double rightDesiredEncoderPos = angle/Constants.Arm.RIGHT_ENCODER_UNIS_PER_ANGLE;
    
    leftArmBelt.set(ControlMode.Position, leftDesiredEncoderPos);
    rightArmBelt.set(ControlMode.Position, rightDesiredEncoderPos);
  } 

  public void reset() {
    boolean leftStatus, rightStatus;
    leftStatus = !leftLimitSwitch.get();
    rightStatus = !rightLimitSwitch.get();
    while (leftStatus && rightStatus) {
      moveDown();
    }
  }

  public void moveDown() {
    if ((!leftLimitSwitch.get()) && (!rightLimitSwitch.get())) {
      leftArmBelt.set(ControlMode.PercentOutput, -1);
    }
  }
  // public void toggleCompressor(){
  // if(!compressorOn){
  // compressor.enableAnalog(2, 120);
  // compressorOn = true;
  // return;
  // }
  // compressor.disable();
  // compressorOn = false;
  // }
  // }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  // public boolean exampleCondition() {
  // // Query some boolean state, such as a digital sensor.
  // return false;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IS EXTENDED?", leftArm.extendTest() && rightArm.extendTest());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
