package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.pneumatics.Pneumatics;

public class GripperIOSolenoid implements GripperIO {
  private static GripperIOSolenoid instance;
  private static DoubleSolenoid solenoid;

  private GripperIOSolenoid() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, GripperConstants.GRIPPER_DOUBLE_SOLENOID_PORTS[0], GripperConstants.GRIPPER_DOUBLE_SOLENOID_PORTS[1]);
    solenoid.set(Value.kReverse);
  }

  public static GripperIOSolenoid getInstance(){
    if(instance == null){
      instance = new GripperIOSolenoid();
    }
    
    return instance;
  }
  
  @Override
  public void toggleSolenoid(){
    solenoid.toggle();
  }

  @Override
  public boolean isClosed() {
    return solenoid.get() == Value.kForward;
  }

  @Override
  public void solenoidForward(){
    solenoid.set(Value.kForward);
    // Pneumatics.getInstance().disableCompressor();
  }

  @Override
  public void solenoidBack(){
    solenoid.set(Value.kReverse);
    Pneumatics.getInstance().enableCompressor();
  }
  
  @Override
  public void solenoidOff(){
    solenoid.set(Value.kOff);
  }
}
