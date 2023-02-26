package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticsIO {
  @AutoLog
  public static class PneumaticsIOInputs {
    boolean isEnabled = false;
  }

  public default void updateInputs(PneumaticsIOInputs inputs) {
  }

  public default void enableCompressor() {
  }

  public default void disableCompressor() {
  }

}
