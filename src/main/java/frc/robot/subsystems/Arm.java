package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// NO
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pneumatics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Arm extends SubsystemBase {

  TalonFX leftBelt, rightBelt;
  TalonSRX wristMotor;
  Pneumatics arms, claw;
  Compressor compressor;
  boolean compressorOn = false;
  DigitalInput shoulderLimitSwitch;

  double leftBeltOffset, rightBeltOffset, wristOffset;

  public Arm() {

    arms = new Pneumatics(Constants.Arm.ARM_FORWARD_CHANNEL, Constants.Arm.ARM_REVERSE_CHANNEL);
    claw = new Pneumatics(Constants.Arm.CLAW_FORWARD_CHANNEL, Constants.Arm.CLAW_REVERSE_CHANNEL);

    compressor = new Compressor(Constants.Arm.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
    leftBelt = new TalonFX(Constants.Arm.LEFT_ARM_MOTOR_ID);
    rightBelt = new TalonFX(Constants.Arm.RIGHT_ARM_MOTOR_ID);

    wristMotor = new TalonSRX(Constants.Arm.WRIST_MOTOR_ID);

    wristMotor.setInverted(true);

    shoulderLimitSwitch = new DigitalInput(Constants.Arm.SHOULDER_LIMIT_SWITCH_ID);


    leftBelt.setInverted(Constants.Arm.LEFT_ARM_MOTOR_INVERTED);
    rightBelt.setInverted(Constants.Arm.RIGHT_ARM_MOTOR_INVERTED);


    leftBelt.setNeutralMode(NeutralMode.Brake);
    rightBelt.setNeutralMode(NeutralMode.Brake);

    compressor.enableDigital();
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

  public void toggleClaw() {
    claw.solenoidToggle();
  }

  public void resetElbow() {
    while(leftBelt.getSensorCollection().isRevLimitSwitchClosed() == 0) moveElbowIn();
    stopElbow();
    leftBeltOffset = leftBelt.getSelectedSensorPosition();
    rightBeltOffset = rightBelt.getSelectedSensorPosition();
  }

  public void resetWrist() {
    while (wristMotor.getSensorCollection().isRevLimitSwitchClosed()) moveWristUp();
    stopWrist();
    wristOffset = wristMotor.getSelectedSensorPosition();
  }

  public void moveElbowOut() {
    leftBelt.set(ControlMode.PercentOutput, .1);
    rightBelt.set(ControlMode.PercentOutput, .1);
    SmartDashboard.putBoolean("BP", true);
  }

  public void moveElbowIn() {
    leftBelt.set(ControlMode.PercentOutput, -.1);
    rightBelt.set(ControlMode.PercentOutput, -.1);
    SmartDashboard.putBoolean("BP", true);
  }

  public void stopElbow() {
    leftBelt.set(ControlMode.PercentOutput, 0);
    rightBelt.set(ControlMode.PercentOutput, 0);
  }

  public void setArm(double deg) {
    leftBelt.set(ControlMode.Position, (deg * Constants.Arm.ENCODER_UNITS_PER_ROTATION_ELBOW) - leftBeltOffset);
    rightBelt.set(ControlMode.Position, (deg * Constants.Arm.ENCODER_UNITS_PER_ROTATION_ELBOW) - rightBeltOffset);
  }

  public void toggleShoulder() {
    arms.solenoidToggle();
  }

  public void moveWristUp() {
    wristMotor.set(ControlMode.PercentOutput, .4);
  }

  public void moveWristDown() {
    wristMotor.set(ControlMode.PercentOutput, -.4);
  }

  public void setWrist(double deg) {
    wristMotor.set(ControlMode.Position, wristOffset + (deg * Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST));
    SmartDashboard.putNumber("wrist target", wristOffset + (deg * Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST));
  }

  public void stopWrist() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

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


  @Override
  public void periodic() {
    SmartDashboard.putNumber("LEFT BELT", leftBelt.getSelectedSensorPosition());
    SmartDashboard.putNumber("RIGHT BELT", leftBelt.getSelectedSensorPosition());
    SmartDashboard.putNumber("WRIST", wristMotor.getSelectedSensorPosition());

    SmartDashboard.putBoolean("BELT LIMIT", leftBelt.getSensorCollection().isRevLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("WRIST LIMIT", wristMotor.getSensorCollection().isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("SHOULDER WRIST", !shoulderLimitSwitch.get());

    if(wristMotor.getSensorCollection().isRevLimitSwitchClosed()) {
      wristOffset = wristMotor.getSelectedSensorPosition();
      SmartDashboard.putNumber("wrist offset", wristOffset);
    }

    if(leftBelt.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      stopElbow();
      leftBeltOffset = leftBelt.getSelectedSensorPosition();
      rightBeltOffset = rightBelt.getSelectedSensorPosition();
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
