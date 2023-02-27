// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Tank;

public class tankDrive extends CommandBase {
  private static Tank tank;  /** Creates a new tankDrive. */
  public tankDrive() {
    tank = Tank.getinstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tank);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tank.setLeftSpeed(0);
    tank.setRightSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speedX = -RobotContainer.getRawAxis(0);
    double speedY = -RobotContainer.getRawAxis(1);
    if(speedX < 0.2 && speedX > -0.2){ speedX = 0;}
    if(speedY < 0.2 && speedY > -0.2){ speedY = 0;}
    speedX = speedX/3;
    speedY = speedY/3;
    tank.setLeftSpeed(speedY - speedX);
    tank.setRightSpeed(speedY + speedX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
