package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Pneumatics {
  DoubleSolenoid doubleSolenoid;
  boolean isExtended = false;

  public Pneumatics(int forwardChannel, int reverseChannel) {
      doubleSolenoid = new DoubleSolenoid(42, PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
      doubleSolenoid.set(Value.kOff);
  }

  public void extendSolenoid() {
      doubleSolenoid.set(Value.kReverse);
  }

  public void retractSolenoid() {
      doubleSolenoid.set(Value.kForward);
  }

  public void disableSolenoid() {
      doubleSolenoid.set(Value.kOff);
  }

  public void solenoidToggle(){
    if(isExtended){
      retractSolenoid();
      isExtended = false;
      return;
    }
    extendSolenoid();
    isExtended = true;
  }

  public boolean extendTest(){
    return isExtended;
  }

  }