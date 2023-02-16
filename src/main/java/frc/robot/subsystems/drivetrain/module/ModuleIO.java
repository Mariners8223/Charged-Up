package frc.robot.subsystems.drivetrain.module;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityPerSecRad = 0.0;
    
    public double steeringAbsolutePositionRad = 0.0;
    public double steeringPositionRad = 0.0;
    public double steeringVelocityPerSecRad = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setSteeringBreakMode(boolean enable) {}

  public default void setTurnBrakeMode(boolean enable) {}
}
