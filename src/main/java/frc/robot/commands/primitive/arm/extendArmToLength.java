// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class extendArmToLength extends CommandBase {
  private static Arm arm;
  private double length;
  /** Creates a new extendArmToLength. */
  public extendArmToLength(double length) {
    arm = Arm.getInstance();
    this.length = length;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.extendToLengthMeters(length);
    SmartDashboard.putString("extension command","does this work");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("extension command","it does");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("extension command","it did");
    arm.stopExtensionMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtExtensionSetpoint();
  }
}
