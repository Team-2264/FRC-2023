package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
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
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Arm extends SubsystemBase {

  TalonFX leftBelt, rightBelt;
  TalonSRX wristMotor;
  Pneumatics arms, claw;
  Compressor compressor;
  boolean compressorOn = false;
  DigitalInput shoulderLimitSwitch;

  String wristStatus;

  public Arm() {

    arms = new Pneumatics(Constants.Arm.ARM_FORWARD_CHANNEL, Constants.Arm.ARM_REVERSE_CHANNEL);
    claw = new Pneumatics(Constants.Arm.CLAW_FORWARD_CHANNEL, Constants.Arm.CLAW_REVERSE_CHANNEL);

    compressor = new Compressor(Constants.Arm.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
    leftBelt = new TalonFX(Constants.Arm.LEFT_ARM_MOTOR_ID);
    rightBelt = new TalonFX(Constants.Arm.RIGHT_ARM_MOTOR_ID);

    wristMotor = new TalonSRX(Constants.Arm.WRIST_MOTOR_ID);
    wristMotor.setInverted(true);

    shoulderLimitSwitch = new DigitalInput(Constants.Arm.SHOULDER_LIMIT_SWITCH_ID);

    leftBelt.setNeutralMode(NeutralMode.Brake);
    rightBelt.setNeutralMode(NeutralMode.Brake);

    rightBelt.follow(leftBelt);

    leftBelt.setInverted(Constants.Arm.LEFT_ARM_MOTOR_INVERTED);
    rightBelt.setInverted(InvertType.OpposeMaster);

    leftBelt.configClearPositionOnLimitR(true, 0);

    // rightBelt.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalon, LimitSwitchNormal.NormallyOpen, 30, 0);

    compressor.enableDigital(); 
    
    // PID

    leftBelt.configFactoryDefault();

    leftBelt.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		leftBelt.configNeutralDeadband(0.001);
    leftBelt.setSensorPhase(false);

    leftBelt.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		leftBelt.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    /* Set the peak and nominal outputs */
		leftBelt.configNominalOutputForward(0, 0);
		leftBelt.configNominalOutputReverse(0, 0);
		leftBelt.configPeakOutputForward(1, 0);
		leftBelt.configPeakOutputReverse(-1, 0);

    // leftBelt.supplycu(30); //Peak Current 25% motor Max
    // leftBelt.configPeakCurrentDuration(150);
    // leftBelt.configContinuousCurrentLimit(20); //Stall current 15% motor Max

		/* Set Motion Magic gains in slot0 - see documentation */
		leftBelt.selectProfileSlot(0, 0);
		leftBelt.config_kF(0, .1, 0);
		leftBelt.config_kP(0, , 0);
		leftBelt.config_kI(0, , 0);
		leftBelt.config_kD(0, , 0);

		/* Set acceleration and vcruise velocity - see documentation */
		leftBelt.configMotionCruiseVelocity(15000, 0);
		leftBelt.configMotionAcceleration(6000, 0);

    leftBelt.config_IntegralZone(0, 30);
    
  }

  public void init() {
    moveElbowIn();
    moveWristUp();
    bringShoulderIn(); 
  }


  // COMBINED MOVEMENTS (THESE SHOULD BE THE ONLY ONES CALLED)

  public void bringArmHome() {
    closeElbow();
    closeWrist();
    bringShoulderIn();
  }

  public void setArmIntake() {
    setElbow(90);
    openWrist();
    takeShoulderOut();
    openClaw();
  }

  public void setArmLow() {
    setElbow(90);
    openWrist();
    takeShoulderOut();
  }

  public void setArmHigh() {
    setElbow(120);
    openWrist();
    takeShoulderOut();
  }
  
  // SOLENOID

  private void toggleClaw() {
    claw.solenoidToggle();
  }

  private void openClaw() {
    if(!claw.extendTest()) claw.solenoidToggle();
  }

  private void closeClaw() {
    if(claw.extendTest()) claw.solenoidToggle();
  }

  private void toggleShoulder() {
    arms.solenoidToggle();
  }

  private void bringShoulderIn() {
    if(!shoulderLimitSwitch.get()) arms.solenoidToggle();
  }

  private void takeShoulderOut() {
    if(shoulderLimitSwitch.get()) arms.solenoidToggle();
  }

  // 

  private void toggleWrist() {
    if(wristStatus == "CLOSED") {
      setWrist(90);
      wristStatus = "OPEN";
    } else {
      setWrist(0);
      wristStatus = "CLOSED";
    }
  }

  private void closeWrist() {
    setWrist(0);
    wristStatus = "CLOSED";
  }

  private void openWrist() {
    setWrist(90);
    wristStatus = "OPEN";
  }

  private void closeElbow() {
    setElbow(0);
  }

  // INDIVIDUAL MOVEMENTS 
  // SHOULD ALWAYS BE USING SETPOS

  private void moveElbowOut() {
    leftBelt.set(ControlMode.PercentOutput, .1);
    SmartDashboard.putBoolean("BP", true);
  }

  private void moveElbowIn() {
    leftBelt.set(ControlMode.PercentOutput, -.1);
    SmartDashboard.putBoolean("BP", true);
  }

  private void stopElbow() {
    leftBelt.set(ControlMode.PercentOutput, 0);
  }

  private void setElbow(double deg) {
    leftBelt.set(ControlMode.MotionMagic, deg * Constants.Arm.ENCODER_UNITS_PER_DEGREE_ELBOW);
  }

  private void moveWristDown() {
    wristMotor.set(ControlMode.PercentOutput, .4);
  }

  private void moveWristUp() {
    wristMotor.set(ControlMode.PercentOutput, -.4);
  }

  private void setWrist(double deg) {
    wristMotor.set(ControlMode.Position, deg * Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST);
    SmartDashboard.putNumber("wrist target", deg * Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST);
  }

  private void stopWrist() {
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
    SmartDashboard.putNumber("RIGHT BELT", rightBelt.getSelectedSensorPosition());
    SmartDashboard.putNumber("WRIST", wristMotor.getSelectedSensorPosition());

    SmartDashboard.putBoolean("BELT LIMIT", leftBelt.getSensorCollection().isRevLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("WRIST LIMIT", wristMotor.getSensorCollection().isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("SHOULDER LIMIT", !shoulderLimitSwitch.get());

    if(wristMotor.getSensorCollection().isRevLimitSwitchClosed()) {
      wristMotor.setSelectedSensorPosition(0);
      wristMotor.configForwardSoftLimitThreshold(120 * Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST);
      wristStatus = "CLOSED";
    }

    if(leftBelt.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      leftBelt.configForwardSoftLimitThreshold(180 * Constants.Arm.ENCODER_UNITS_PER_DEGREE_ELBOW);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
