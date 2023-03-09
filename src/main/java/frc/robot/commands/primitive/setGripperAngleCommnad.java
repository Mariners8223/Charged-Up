package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class setGripperAngleCommnad extends CommandBase {
  private Gripper gripper;
  private double desiredAngleDeg;

  public setGripperAngleCommnad(double desiredAngleDeg) {
    this.gripper = Gripper.getInstance();
    this.desiredAngleDeg = desiredAngleDeg;

    addRequirements(gripper);
  }

  @Override
  public void initialize() {
    gripper.setAngle(desiredAngleDeg);
  }

  @Override
  public void end(boolean interrupted) {
    gripper.stop();
  }

  @Override
  public boolean isFinished() {
    return gripper.isAtSetpoint();
  }
}
