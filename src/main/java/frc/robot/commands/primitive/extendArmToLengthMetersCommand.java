package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class extendArmToLengthMetersCommand extends CommandBase {
  private Arm arm;
  private double desiredLengthMeters;
  public extendArmToLengthMetersCommand(double desiredLengthMeters) {
    arm = Arm.getInstance();
    this.desiredLengthMeters = desiredLengthMeters+ this.desiredLengthMeters;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.extendToLengthMeters(desiredLengthMeters);
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopExtensionMotor();
  }

  @Override
  public boolean isFinished() {
    return arm.isAtExtensionSetpoint();
  }
}
