package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  @AutoLog
  public static class GripperIOInputs {
    boolean isClosed = false;
    double velocityPerSecDeg = 0.0;
    double gripperDeg = 0.0;
  }

  public default void updateInputs(GripperIOInputs inputs) {
  }
  
  public default void setVoltage(double voltage) {
  }
}
