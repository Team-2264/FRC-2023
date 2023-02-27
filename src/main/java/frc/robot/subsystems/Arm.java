package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// NO
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pneumatics;
import frc.robot.enums.ArmStatus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.Joystick;

public class Arm extends SubsystemBase {

  public TalonFX leftBelt, rightBelt;
  TalonSRX wristMotor;
  Pneumatics arms, claw;
  Compressor compressor;
  boolean compressorOn = false;
  DigitalInput shoulderLimitSwitch;

  public double armPosition, wristPosition, startTime;
  boolean canUseArm, canUseWrist;
  int i;

  private ArmStatus status;

  private Joystick arm;

  StringBuilder _sb = new StringBuilder();

  public Arm(Joystick input) {

    arm = input;

    status = ArmStatus.HOME;
    armPosition = 0;
    wristPosition = 0;
    i = 0;

    canUseArm = false;
    canUseWrist = false;

    arms = new Pneumatics(Constants.Arm.ARM_FORWARD_CHANNEL, Constants.Arm.ARM_REVERSE_CHANNEL);
    claw = new Pneumatics(Constants.Arm.CLAW_FORWARD_CHANNEL, Constants.Arm.CLAW_REVERSE_CHANNEL);

    compressor = new Compressor(Constants.Arm.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
    leftBelt = new TalonFX(Constants.Arm.LEFT_ARM_MOTOR_ID);
    rightBelt = new TalonFX(Constants.Arm.RIGHT_ARM_MOTOR_ID);

    leftBelt.configFactoryDefault();
    rightBelt.configFactoryDefault();

    wristMotor = new TalonSRX(Constants.Arm.WRIST_MOTOR_ID);
    wristMotor.setInverted(true);

    wristMotor.configClearPositionOnLimitR(true, 0);

    wristMotor.configFactoryDefault();

    shoulderLimitSwitch = new DigitalInput(Constants.Arm.SHOULDER_LIMIT_SWITCH_ID);

    leftBelt.setNeutralMode(NeutralMode.Brake);
    rightBelt.setNeutralMode(NeutralMode.Brake);

    leftBelt.setInverted(false);

    rightBelt.follow(leftBelt);

    rightBelt.setInverted(true);

    leftBelt.configClearPositionOnLimitR(false, 0);

    compressor.enableDigital();

    /* Configure Sensor Source for Pirmary PID */
    leftBelt.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    // // PID

    leftBelt.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
    leftBelt.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    leftBelt.configNominalOutputForward(0, 10);
    leftBelt.configNominalOutputReverse(0, 10);
    leftBelt.configPeakOutputForward(1, 10);
    leftBelt.configPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    leftBelt.selectProfileSlot(0, 0);
    leftBelt.config_kF(0, .047, 10);
    leftBelt.config_kP(0, 0.6, 10);
    leftBelt.config_kI(0, 0, 10);
    leftBelt.config_kD(0, 13, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    leftBelt.configMotionCruiseVelocity(750, 10);
    leftBelt.configMotionAcceleration(1000, 10);

    leftBelt.configNeutralDeadband(0.05, 10);

    // // PID

    wristMotor.config_kP(0, 4, 10);
    wristMotor.config_kI(0, 0, 10);
    wristMotor.config_kD(0, 40, 10);

    wristMotor.configNeutralDeadband(0.001, 10);

    leftBelt.configNeutralDeadband(0.01, 10);

  }

  public void init() {
    moveElbowIn();
    moveWristUp();
    bringShoulderIn();

    canUseArm = false;
    canUseWrist = false;
  }

  public void adjustWrist(double amounts) {
    wristPosition = wristPosition + amounts;
    SmartDashboard.putNumber("WRIST", wristPosition);
    SmartDashboard.putNumber("WRIST AFTER", amounts);
  }

  // COMBINED MOVEMENTS (THESE SHOULD BE THE ONLY ONES CALLED)

  public void bringArmHome() {
    bringShoulderIn();
    setArmHome();
    setWristHome();
    closeClaw();

    status = ArmStatus.HOME;
  }

  public void setLowIntake() {
    setArmLowIntake();
    setWristAngledDown();
    openClaw();
    bringShoulderIn();

    status = ArmStatus.INTAKE_LOW;
  }

  public void setLow() {
    setArmLow();
    setWristAngledDownMore();
    bringShoulderIn();
    openClaw();

    status = ArmStatus.LOW;
  }

  public void setMidCube() {
    setArmMidCube();
    setWristFlat();
    bringShoulderIn();

    startTime = System.currentTimeMillis();
    status = ArmStatus.CUBE_MID;
  }

  public void setMidCone() {
    setArmMidCone();
    setWristMid();
    bringShoulderIn();

    startTime = System.currentTimeMillis();
    status = ArmStatus.CONE_MID;
  }

  public void intake() {
    bringShoulderIn();
    setArmHigh();
    setWristFlat();
    openClaw();

    status = ArmStatus.INTAKE;
  }

  public void intermediateAuto() {
    bringShoulderIn();
    setArmHigh();
    setWristHome();

    status = ArmStatus.INTERMEDIATE;
  }

  public void simbaCone() {
    setWristHigh();
    setArmHighestCone();
    takeShoulderOut();

    startTime = System.currentTimeMillis();
    status = ArmStatus.CONE_SIMBA;
  }

  public void simbaCube() {
    setWristMid();
    setArmHighestCube();
    takeShoulderOut();

    startTime = System.currentTimeMillis();
    status = ArmStatus.CUBE_SIMBA;
  }

  public ArmStatus getStatus() {
    return status;
  }

  // SETPOINTS

  private void setArmHome() {
    armPosition = 0;
  }

  private void setArmLowIntake() {
    armPosition = 5;
  }

  private void setArmLow() {
    armPosition = 15;
  }

  private void setArmMidCube() {
    armPosition = 54;
  }

  private void setArmMidCone() {
    armPosition = 70;
  }

  private void setArmHigh() {
    armPosition = 70;
  }

  private void setArmHighestCone() {
    armPosition = 142;
  }

  private void setArmHighestCube() {
    armPosition = 130;
  }

  private void setWristHome() {
    wristPosition = 0;
  }

  private void setWristFlat() {
    wristPosition = 57;
  }

  private void setWristAngledDown() {
    wristPosition = 130;
  }

  private void setWristAngledDownMore() {
    wristPosition = 165;
  }

  private void setWristMid() {
    wristPosition = 30;
  }

  private void setWristHigh() {
    wristPosition = 0;
  }

  // SETFUNCTIONS

  private void setElbow(double deg) {
    leftBelt.set(ControlMode.MotionMagic, deg * Constants.Arm.ENCODER_UNITS_PER_DEGREE_ELBOW,
        DemandType.ArbitraryFeedForward, 0.07 * Math.cos(deg * (Math.PI / 180)));
  }

  private void setWrist(double deg) {
    wristMotor.set(ControlMode.Position, deg * Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST);
  }

  // RAW MOVEMENTS

  private void moveElbowIn() {
    leftBelt.set(ControlMode.PercentOutput, -.15);
  }

  public void moveWristUp() {
    wristMotor.set(ControlMode.PercentOutput, -.2);
  }

  // SOLENOID

  public void toggleClaw() {
    claw.solenoidToggle();
  }

  public void openClaw() {
    if (!claw.isExtended())
      claw.solenoidToggle();
  }

  public void closeClaw() {
    if (claw.isExtended())
      claw.solenoidToggle();
  }

  private void toggleShoulder() {
    arms.solenoidToggle();
  }

  private void takeShoulderOut() {
    if (!arms.isExtended())
      arms.solenoidToggle();
  }

  private void bringShoulderIn() {
    if (arms.isExtended())
      arms.solenoidToggle();
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
    SmartDashboard.putNumber("LEFT BELT",
        leftBelt.getSelectedSensorPosition() / Constants.Arm.ENCODER_UNITS_PER_DEGREE_ELBOW);
    SmartDashboard.putNumber("RIGHT BELT", rightBelt.getSelectedSensorPosition());
    SmartDashboard.putNumber("WRIST",
        wristMotor.getSelectedSensorPosition() / Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST);

    SmartDashboard.putBoolean("BELT LIMIT", leftBelt.getSensorCollection().isRevLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("WRIST LIMIT", wristMotor.getSensorCollection().isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("SHOULDER LIMIT", !shoulderLimitSwitch.get());

    if (wristMotor.getSensorCollection().isRevLimitSwitchClosed() && !canUseWrist) {
      wristMotor.setSelectedSensorPosition(0);
      setWristHome();
      // wristMotor.configForwardSoftLimitThreshold(650);

      canUseWrist = true;
    }

    if (leftBelt.getSensorCollection().isRevLimitSwitchClosed() == 1 && !canUseArm) {
      leftBelt.setSelectedSensorPosition(0);
      setArmHome();
      // leftBelt.configForwardSoftLimitThreshold(8500);
      canUseArm = true;
    }

    if (canUseArm) {
      setElbow(armPosition);
    }

    if (canUseWrist && (System.currentTimeMillis() - startTime > 1250)) {
      setWrist(wristPosition);
    }

    i++;

    if (i % 20 == 0) {
      _sb.append("\tBELT Out%:");
      _sb.append(leftBelt.getMotorOutputPercent());
      _sb.append("\tVel:");
      _sb.append(leftBelt.getSelectedSensorVelocity());

      /* Append more signals to print when in speed mode */
      _sb.append("\tposition:");
      _sb.append(wristMotor.getSelectedSensorPosition() / Constants.Arm.ENCODER_UNITS_PER_DEGREE_ELBOW);
      _sb.append("\terr:");
      _sb.append(leftBelt.getClosedLoopError());
      _sb.append("\ttrg:");
      _sb.append(armPosition);

      // System.out.println(_sb);
      _sb.setLength(0);

      _sb.append("\tWRIST Out%:");
      _sb.append(wristMotor.getMotorOutputPercent());
      _sb.append("\tVel:");
      _sb.append(wristMotor.getSelectedSensorVelocity());

      /* Append more signals to print when in speed mode */
      _sb.append("\tposition:");
      _sb.append(wristMotor.getSelectedSensorPosition() / Constants.Arm.ENCODER_UNITS_PER_DEGREE_WRIST);
      _sb.append("\terr:");
      _sb.append(wristMotor.getClosedLoopError());
      _sb.append("\ttrg:");
      _sb.append(wristPosition);

      // System.out.println(_sb);
      _sb.setLength(0);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
