// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.primitive.arm.RotateArmToPoint;
import frc.robot.commands.primitive.arm.extendArmToLength;
import frc.util.SequenceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToSetPoint extends SequentialCommandGroup {
  private SequenceType lastPostion;
  private SequenceType postion = SequenceType.Orienation;
  /** Creates a new MoveArmToSetPoint. */
  public MoveArmToSetPoint(boolean state) {
    //I know this is a stupid way of implneting it
    //the point is to make the postions reletive to the last postion
    //Alon's idea
    if(state){
      switch(lastPostion){
        case Orienation:
          postion = SequenceType.GridFloor;
          break;
        
        case GridFloor:
          postion = SequenceType.GridLevel1;
          break;

        case GridLevel1:
          postion = SequenceType.GridLevel2;
          break;
        
        case GridLevel2:
          postion = SequenceType.GridLevel2;
          break;
      }
    }
    else{
      switch(lastPostion){
        case Orienation:
          postion = SequenceType.Orienation;
          break;
        
        case GridFloor:
          postion = SequenceType.Orienation;
          break;

        case GridLevel1:
          postion = SequenceType.GridFloor;
          break;
        
        case GridLevel2:
          postion = SequenceType.GridLevel1;
          break;
      }
    }


    switch(postion){
      
      case Orienation:
        addCommands
        (new extendArmToLength(0),
        new RotateArmToPoint(0),
        new extendArmToLength(20));
        lastPostion = SequenceType.Orienation;
        break;
      
      case GridFloor:
        addCommands
        (new extendArmToLength(0),
        new RotateArmToPoint(30),
        new extendArmToLength(5));
        lastPostion = SequenceType.GridFloor;
        break;
      
      case GridLevel1:
        addCommands
        (new extendArmToLength(0),
        new RotateArmToPoint(117),
        new extendArmToLength(2));
        lastPostion = SequenceType.GridLevel1;
        break;

      case GridLevel2:
        addCommands
        (new extendArmToLength(0),
        new RotateArmToPoint(133),
        new extendArmToLength(51));
        lastPostion = SequenceType.GridLevel2;
        break;
    }
  }
}
