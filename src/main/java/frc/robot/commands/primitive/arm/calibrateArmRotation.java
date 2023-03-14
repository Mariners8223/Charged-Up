// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class calibrateArmRotation extends CommandBase {
  private static Arm arm;
  private boolean toggled;
  /** Creates a new calibrateArmRotation. */
  public calibrateArmRotation() {
    arm = Arm.getInstance();
    toggled = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!toggled){
      arm.setFalconPO(-0.3);
      toggled = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopRotationMotor();
    arm.resetRotationEncoder(-2);
    arm.rotateToAngleDegrees(0);
    arm.setRotationCalibrated(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getRotationLimitSwitch() & arm.isExtensionCalibrated();
  }
}
