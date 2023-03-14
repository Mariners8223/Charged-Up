// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.arm;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class MoveArmToSetPoint extends CommandBase {
  private static Arm arm;
  double rotation;
  double extension;
  boolean roateted;
  /** Creates a new MoveArmToSetPoint. */
  public MoveArmToSetPoint() {
    arm = Arm.getInstance();
    rotation = 0;
    extension = 0;
    roateted = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int position = RobotContainer.getArmPosition();
    roateted = false;
    extension = 0;
    rotation = 0;
    String currentpostion = "shit";
    switch(position){
      case 0:
        rotation = 5;
        extension = 17;
        currentpostion = "Home";
        break;
      
      case 1:
        rotation = 50;
        extension = 19;
        currentpostion = "Grid Floor ";
        break;

      case 2:
        rotation = 120;
        extension = 10;
        currentpostion = "Grid Middle";
        break;

      case 3:
        rotation = 133;
        extension = 47;
        currentpostion = "Grid Top";
        break;

      case 4:
        rotation = 120;
        extension = 40;
        currentpostion = "Double SubStation";
        break;   
    }
    SmartDashboard.putString("current Postion", currentpostion);
    arm.extendToLengthMeters(0);
    
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
    return arm.isAtExtensionSetpoint() && arm.isAtRotationSetpoint() && roateted & arm.isExtensionCalibrated();
  }
}
