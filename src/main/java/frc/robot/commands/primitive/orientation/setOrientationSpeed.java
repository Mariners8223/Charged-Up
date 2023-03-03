package frc.robot.commands.primitive.orientation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class setOrientationSpeed extends CommandBase {
  Orientation orientation;

  public setOrientationSpeed() {
    orientation = Orientation.getInstance();
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    orientation.lowerOrientation();
    orientation.lowerRamp();
    orientation.setSpeed(0.6);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orientation.stop();
    orientation.raiseOrientation();
    orientation.raiseRamp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
