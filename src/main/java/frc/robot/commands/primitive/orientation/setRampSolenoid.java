package frc.robot.commands.primitive.orientation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class setRampSolenoid extends CommandBase {
  Orientation orientation;
  boolean raiseRamp;
  public setRampSolenoid(boolean raiseRamp) {
    orientation = Orientation.getInstance();
    this.raiseRamp = raiseRamp;
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (raiseRamp) orientation.raiseRamp();
    else orientation.lowerRamp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
