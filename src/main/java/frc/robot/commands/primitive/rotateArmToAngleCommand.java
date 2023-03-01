package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class rotateArmToAngleCommand extends CommandBase {
  private Arm arm;
  private double desiredAngleDeg;
  public rotateArmToAngleCommand(double desiredAngleDeg) {
    arm = Arm.getInstance();
    this.desiredAngleDeg = desiredAngleDeg;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.rotateToAngleDegrees(desiredAngleDeg);
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
