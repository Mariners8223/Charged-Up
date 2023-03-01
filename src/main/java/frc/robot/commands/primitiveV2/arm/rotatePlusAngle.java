// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitiveV2.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class rotatePlusAngle extends CommandBase {
  private static Arm arm;
  /** Creates a new rotatePlusAngle. */
  public rotatePlusAngle() {
    arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.stopRotationMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.getRawAxis(5);
    if(speed < 0.2 && speed > -0.2){
      speed = 0;
    }
    if(RobotContainer.getInverted()){ speed = -speed;}
    arm.rotatePlusAbgleDegrees(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopRotationMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
