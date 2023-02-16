package frc.robot.subsystems.drivetrain.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public double positionDeg = 0.0;
    public double robotPitch = 0.0;
    public boolean isConnected = false;
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }

  public default boolean isConnected() { return false; }

}