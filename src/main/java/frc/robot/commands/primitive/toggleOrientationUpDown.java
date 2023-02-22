// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class toggleOrientationUpDown extends CommandBase {
  private Orientation orientation;

  public toggleOrientationUpDown() {
    this.orientation = Orientation.getInstance();

    addRequirements(orientation);
  }

  @Override
  public void initialize() {
    orientation.toggleSolenoid();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}