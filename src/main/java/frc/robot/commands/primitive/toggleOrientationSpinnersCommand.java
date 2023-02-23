// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;
import frc.util.SequenceType;

public class toggleOrientationSpinnersCommand extends CommandBase {
  private Orientation orientation;
  private double speed;

  public toggleOrientationSpinnersCommand(double speed) {
    this.orientation = Orientation.getInstance();
    this.speed = speed;

    addRequirements(orientation);
  }

  @Override
  public void initialize() {
    orientation.setSpeed(speed);
    orientation.toggleSolenoid();
    orientation.setIsRunning(!orientation.getIsRunning());
  }
  @Override
  public void end(boolean interrupted) {
    orientation.setSpeed(0);
    orientation.toggleSolenoid();
    orientation.setIsRunning(!orientation.getIsRunning());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
