package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  Servo leftArmServo = new Servo(0);
  Servo rightArmServo = new Servo(1);

  public Arm() {
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
