package frc.robot.Commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class rotateArmToAngleCommand extends CommandBase {
  private Arm arm;
  private double desiredAngle;
  public rotateArmToAngleCommand(double desiredAngle) {
    arm = Arm.getInstance();
    this.desiredAngle = desiredAngle;
  }

  @Override
  public void initialize() {
    arm.rotateToAngleDegrees(desiredAngle);
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopRotationMotor();
  }

  @Override
  public boolean isFinished() {
    return arm.isAtRotationSetpoint();
  }
}
