package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  @AutoLog
  public static class GripperIOInputs {
    public boolean isClosed = true;
  }

  public default void toggleSolenoid(){
  }

  public default boolean isClosed() {
    return true;
  }

  public default void solenoidForward(){
  }

  public default void solenoidBack(){
  }
  
  public default void solenoidOff(){
  }

}
