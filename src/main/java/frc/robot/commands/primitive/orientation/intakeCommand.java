package frc.robot.commands.primitive.orientation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class intakeCommand extends CommandBase {
  Orientation orientation;
  double deltaTime;
  public intakeCommand() {
    orientation = Orientation.getInstance();
  }

  @Override
  public void initialize() {
    deltaTime = Timer.getFPGATimestamp();
    orientation.lowerRamp();
    orientation.lowerOrientation();
    orientation.setSpeed(0.6);
  }

  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - deltaTime >= 2) {
      orientation.raiseRamp();
      deltaTime = Timer.getFPGATimestamp();
    } else {
      orientation.lowerRamp();
    }
  }

  @Override
  public void end(boolean interrupted) {
    orientation.stop();
    orientation.raiseRamp();
    orientation.raiseOrientation();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}