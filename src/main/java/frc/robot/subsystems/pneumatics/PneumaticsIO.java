package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticsIO {
  @AutoLog
  public static class PneumaticsIOInputs {
    double currentPSI = 0.0;
    boolean isEnabled = false;
  }

  public default void updateInputs(PneumaticsIOInputs inputs) {
  }

  public default void enableCompressor() {
  }

  public default void disableCompressor() {
  }

  public default double getPressure() {
    return 0.0;
  }

}
