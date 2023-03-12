package frc.robot.commands;


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import javax.swing.text.Position;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.orientation.Orientation;

public class MoveArmToSetPoint extends CommandBase {
  private static Arm arm;
  private static Orientation orientation;
  double rotation;
  double extension;
  boolean roateted;
  /** Creates a new MoveArmToSetPoint. */
  public MoveArmToSetPoint() {
    arm = Arm.getInstance();
    orientation = Orientation.getInstance();
    rotation = 0;
    extension = 0;
    roateted = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int position = RobotContainer.getArmPosition();
    roateted = false;
    extension = 0;
    rotation = 0;
    String positionName = "shit";
    switch(position){
      case 0:
        rotation = 0;
        extension = 25;
        orientation.lowerRamp();
        positionName = "Home";
        break;
      
      case 1:
        rotation = 50;
        extension = 15;
        positionName = "Grid Bottom / Floor";
        break;

      case 2:
        rotation = 113;
        extension = 5;
        positionName = "Grid Middle";
        break;

      case 3:
        rotation = 133;
        extension = 51;
        positionName = "Grid top";
        break;

      case 4:
        rotation = 140;
        extension = 51;
        positionName = "Double SubStaion";
        break;   
    }
    
    arm.extendToLengthMeters(0);
    SmartDashboard.putString("Current Arm Postion", positionName);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(arm.isAtExtensionSetpoint()){
      arm.rotateToAngleDegrees(rotation);
      roateted = true;
    }

    if(arm.isAtRotationSetpoint() && roateted){
      arm.extendToLengthMeters(extension);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.isAtExtensionSetpoint() && arm.isAtRotationSetpoint() && roateted) || !arm.isCalibrated();
  }
}
