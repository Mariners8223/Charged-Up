// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Tank;

public class TankDrive extends CommandBase {
  private static Tank tank;
  private static double speedX;
  private static double speedY;
  /** Creates a new TankDrive. */
  public TankDrive() {
    tank = Tank.getinstance();
    speedX = 0.0;
    speedY = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tank);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedX = RobotContainer.getRawAxis(0)/2;
    speedY = RobotContainer.getRawAxis(1)/2;
    if(speedX < 0.1 && speedX > -0.1){
      speedX = 0.0;
    }
    if(speedY < 0.1 && speedY > -0.1){
      speedY = 0.0;
    }
    tank.setRightSpeed(speedY + speedX);
    tank.setLeftSpeed(speedY - speedX);
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
