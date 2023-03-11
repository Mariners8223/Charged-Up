package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ExtendAndRotateCommand extends CommandBase {
  Arm arm;
  private double angle;
  private double length;
  public ExtendAndRotateCommand(double angle, double length) {
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.extendToLengthMeters(length);
    arm.rotateToAngleDegrees(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopExtensionMotor();
    arm.stopRotationMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.isAtExtensionSetpoint() && arm.isAtRotationSetpoint()) || arm.getExtensionLimitSwitch();
  }
}
