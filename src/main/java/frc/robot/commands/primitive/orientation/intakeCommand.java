package frc.robot.commands.primitive.orientation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class intakeCommand extends CommandBase {
  Orientation orientation;
  double speed;
  public intakeCommand(double speed) {
    orientation = Orientation.getInstance();
    this.speed = speed;
  }

  @Override
  public void initialize() {
    System.out.println("the fuck??");
    orientation.lowerRamp();
    orientation.lowerOrientation();
    orientation.setSpeed(speed);
    // Pneumatics.getInstance().disableCompressor();
  }


  @Override
  public void end(boolean interrupted) {
    orientation.stop();
    orientation.raiseRamp();
    orientation.raiseOrientation();
    // Pneumatics.getInstance().enableCompressor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
