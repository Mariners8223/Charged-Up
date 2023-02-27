// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class orienation extends CommandBase {
  private static Orientation orientation;
  /** Creates a new orienation. */
  public orienation() {
    orientation = Orientation.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    orientation.setSpeed(-0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orientation.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}