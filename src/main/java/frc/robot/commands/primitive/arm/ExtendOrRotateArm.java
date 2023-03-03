// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.util.SequenceType;

public class ExtendOrRotateArm extends CommandBase {
  private static Arm arm;
  private double point;
  private SequenceType sequenceType;
  /** Creates a new ExtendOrRotateArm. */
  public ExtendOrRotateArm(SequenceType sequenceType, double point) {
    arm = Arm.getInstance();
    this.sequenceType = sequenceType;
    this.point = point;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(sequenceType){
      case Arm:
        arm.rotateToAngleDegrees(point);
        break;
      
      case Extenion:
        arm.extendToLengthMeters(point);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch(sequenceType){
      case Arm:
        arm.stopRotationMotor();
        break;
      
      case Extenion:
        arm.stopExtensionMotor();
        break;
        
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch(sequenceType){
      case Arm:
        return arm.isAtRotationSetpoint();

      case Extenion:
        return arm.isAtExtensionSetpoint();

      default:
        return false;
    }
  }
}
