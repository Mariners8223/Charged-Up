// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class humenArmAndRotate extends CommandBase {
  private static Arm arm;
  /** Creates a new humenArmAndRotate. */
  public humenArmAndRotate() {
    arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axisX = RobotContainer.getRawAxis(4);
    double axisY = RobotContainer.getRawAxis(5);
    if(axisX < 0.2 && axisX > -0.2){
      axisX = 0.0;
    }
    if(axisY < 0.2 && axisY > -0.2){
      axisY = 0.0;
    }
    axisX = axisX*10;
    axisY = axisY/10;
    arm.extendPlusLengthMeters(axisY);
    arm.rotatePlusAbgleDegrees(axisX);
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
