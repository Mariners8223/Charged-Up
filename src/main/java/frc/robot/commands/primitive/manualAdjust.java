// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class manualAdjust extends CommandBase {
  private static Arm arm;
  private  boolean inverted;
  private boolean motor;
  /** Creates a new manualAdjust. */
  public manualAdjust(boolean motor,boolean inverted) {
    arm = arm.getInstance();
    this.inverted = inverted;
    this.motor = motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(motor){
      double speed = RobotContainer.getRawAxis(4)/2;
      if(inverted){ speed = -speed;}
      arm.setPercentSpeed(motor, speed);
    }
    else{
      double speed = RobotContainer.getRawAxis(5)/2;
      if(inverted){ speed = -speed;}
      arm.setPercentSpeed(motor, speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopExtensionMotor();
    arm.stopRotationMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
