// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.util.SequenceType;

public class manualAdjust extends CommandBase {
  private static Arm arm;
  private SequenceType sequenceType;
  /** Creates a new manualAdjust. */
  public manualAdjust(SequenceType sequenceType) {
    arm = Arm.getInstance();
    this.sequenceType = sequenceType;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(sequenceType){
      case Arm:
        arm.stopRotationMotor();
        break;
      
      case Extenion:
        arm.stopExtensionMotor();
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(sequenceType){
      case Arm:
        double speed = -RobotContainer.getRawAxis(4);

        if(speed < 0.2 && speed > -0.2){
          speed = 0;
        }

        MathUtil.clamp(speed, -1, 1);

        if(RobotContainer.getInverted()){ speed = -speed;}

        arm.rotatePlusAbgleDegrees(speed);
        break;
      
        case Extenion:
        double length = RobotContainer.getRawAxis(3);

        if(length < 0.2 && length > -0.2){ length = 0;}

        MathUtil.clamp(length, -1, 1);

        if(RobotContainer.getInverted()){ length = -length;}

        arm.extendPlusLengthMeters(length);

    }
  }

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
    return false;
  }
}
