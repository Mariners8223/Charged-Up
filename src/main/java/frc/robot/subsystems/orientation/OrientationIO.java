package frc.robot.subsystems.orientation;

import org.littletonrobotics.junction.AutoLog;

public interface OrientationIO {
  @AutoLog
  public static class OrientationIOInputs {
    boolean isOrientaionClosed = true;
    boolean isRampClosed = true;
    boolean isRunning = false;
  }

  public default void updateInputs(OrientationIOInputs inputs) {
  }

  public default void toggleRampSolenoid() {
  }

  public default void toggleUpSolenoid(){
    
  }

  public default void SetRampSolenoidState(){

  }

  public default void SetUpSolenoid(){
    
  }

  public default void setPercent(double percenttop) {
  }

  public default void disableMotors(){

  }
}
