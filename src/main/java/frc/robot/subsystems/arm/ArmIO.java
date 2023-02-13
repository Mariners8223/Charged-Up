package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    double armAngleRad = 0.0;
    double armExtensionMeters = 0.0;
    double armVelocityPerSecRad = 0.0;
  }
  

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {
  }

  public default void moveToAngle(double desiredAngles) {
  }

  public default void extendToLength(double extensionMeters) {
  }
}
