package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pneumatics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  Servo leftArmServo = new Servo(0);
  Servo rightArmServo = new Servo(1);
  Pneumatics doubleTest;
  Compressor compressor;
  boolean compressorOn = false;

  public Arm() {
    doubleTest = new Pneumatics(0, 1);
    compressor = new Compressor(42, PneumaticsModuleType.CTREPCM);
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

  public void closeClaw() {
    leftArmServo.set(0.5);
    rightArmServo.set(0.5);
    System.out.println("Neev and his stupid ass claw");
  }

  public void openClaw() {
    leftArmServo.set(0);
    rightArmServo.set(0);
    System.out.println("Neev and his stupid ass claw but open");
  }
public void expand(){
    doubleTest.extendSolenoid();
  }

  public void retract(){
    doubleTest.retractSolenoid();
  }

  public void disable(){
    doubleTest.disableSolenoid();
  }

  public void toggle(){
    doubleTest.solenoidToggle();
  }

  // public void toggleCompressor(){
  //   if(!compressorOn){
  //     compressor.enableAnalog(2, 120);
  //     compressorOn = true;
  //     return;
  //   }
  //   compressor.disable();
  //   compressorOn = false;
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
  //   // Query some boolean state, such as a digital sensor.
  //   return false;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IS EXTENDED?", doubleTest.extendTest());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
