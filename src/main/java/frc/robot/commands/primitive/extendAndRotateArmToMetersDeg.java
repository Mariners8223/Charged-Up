// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class extendAndRotateArmToMetersDeg extends CommandBase {
  private Arm arm;
  private double desiredAngleDeg;
  private double desiredLengthMeters;

  public extendAndRotateArmToMetersDeg(double desiredAngleDeg, double desiredLengthMeters) {
    this.arm = Arm.getInstance();
    this.desiredAngleDeg = desiredAngleDeg;
    this.desiredLengthMeters = desiredLengthMeters;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.extendToLengthMeters(desiredLengthMeters);
    arm.rotateToAngleDegrees(desiredAngleDeg);
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopExtensionMotor();
    arm.stopRotationMotor();
  }

  @Override
  public boolean isFinished() {
    return arm.isAtExtensionSetpoint()&&arm.isAtRotationSetpoint();
  }
}
