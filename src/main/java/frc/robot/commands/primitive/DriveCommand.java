// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tank;

public class DriveCommand extends CommandBase {
  private Tank tank;
  private double speed;
  private double rot;

  public DriveCommand(double speed, double rot) {
    this.tank = Tank.getInstance();
    this.speed = speed;
    this.rot = rot;

    addRequirements(tank);
    }

  @Override
  public void initialize() {
    tank.Drive(speed, rot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
