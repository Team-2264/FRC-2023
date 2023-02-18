package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Pneumatics {
  DoubleSolenoid doubleSolenoid;
  boolean isExtended;

  public Pneumatics(int forwardChannel, int reverseChannel) {
      doubleSolenoid = new DoubleSolenoid(Constants.Arm.COMPRESSOR_ID, PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
      doubleSolenoid.set(Value.kForward);
      isExtended = false;
  }

  public void extendSolenoid() {
      doubleSolenoid.set(Value.kReverse);
      isExtended = true;
  }

  public void retractSolenoid() {
      doubleSolenoid.set(Value.kForward);
      isExtended = false;
  }

  public void disableSolenoid() {
      doubleSolenoid.set(Value.kOff);
  }

  public void solenoidToggle(){
    if(isExtended){
      retractSolenoid();
      isExtended = false;
    } else {
      extendSolenoid();
      isExtended = true;
    }
  }

  public boolean extendTest(){
    return isExtended;
  }

  }