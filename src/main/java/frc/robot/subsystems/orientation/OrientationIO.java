package frc.robot.subsystems.orientation;

import org.littletonrobotics.junction.AutoLog;

public interface OrientationIO {
  @AutoLog
  public static class OrientationIOInputs {
    boolean isClosed = true;
    boolean isRunning = false;
  }

  public default void updateInputs(OrientationIOInputs inputs) {
  }

  public default void toggleOrientation() {
  }

  public default void setPercent(double percenttop, double percentbottom) {
  }
}
