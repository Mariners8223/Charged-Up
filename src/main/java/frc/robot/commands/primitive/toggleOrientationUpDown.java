package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class toggleOrientationUpDown extends CommandBase {
  private Orientation orientation;

  public toggleOrientationUpDown() {
    this.orientation = Orientation.getInstance();

    addRequirements(orientation);
  }

  @Override
  public void initialize() {
    orientation.toggleSolenoid();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}