package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.orientation.Orientation;

public class calibrateArm extends CommandBase {
  Arm arm;
  public calibrateArm() {
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.set775PO(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopExtensionMotor();
    Orientation.getInstance().raiseRamp();
    arm.resetExtensionEncoder(-0);
    arm.extendToLengthMeters(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getExtensionLimitSwitch();
  }
}
