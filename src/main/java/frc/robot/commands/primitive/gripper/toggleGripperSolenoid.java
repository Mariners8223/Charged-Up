// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.gripper;

public class toggleGripperSolenoid extends CommandBase {
  private static gripper gripperV2;
  /** Creates a new toggleGripperSolenoid. */
  public toggleGripperSolenoid() {
    gripperV2 = gripper.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperV2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripperV2.toggleSolenoid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
