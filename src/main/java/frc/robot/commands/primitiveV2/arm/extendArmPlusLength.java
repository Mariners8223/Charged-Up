// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitiveV2.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class extendArmPlusLength extends CommandBase {
  private static Arm arm;
  /** Creates a new extendArmPlusLength. */
  public extendArmPlusLength() {
    arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.stopExtensionMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double length = RobotContainer.getRawAxis(4);
    if(length < 0.2 && length > -0.2){ length = 0;}
    if(RobotContainer.getInverted()){ length = -length;}
    arm.extendPlusLengthMeters(length);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    arm.stopExtensionMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
