// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.primitive.arm.RotateArmToPoint;
import frc.robot.commands.primitive.arm.extendArmToLength;
import frc.util.SequenceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToSetPoint extends SequentialCommandGroup {
  private SequenceType lastPostion = SequenceType.Orienation;
  /** Creates a new MoveArmToSetPoint. */
  public MoveArmToSetPoint(boolean state) {
    int roation= 0;
    int exnteion = 0;
    if(state){
      switch(lastPostion){
        case Orienation:
          roation = 30;
          exnteion = 15;
          lastPostion = SequenceType.Orienation;
          break;
        
        case GridFloor:
          roation = 107;
          exnteion = 2;
          lastPostion = SequenceType.GridFloor;
          break;
        
        case GridLevel1:
          roation = 130;
          exnteion = 45;
          lastPostion = SequenceType.GridLevel1;
          break;

        case GridLevel2:
          roation = 130;
          exnteion = 45;
          lastPostion = SequenceType.GridLevel2;
          break;
          
      }
    }
    else{
      switch(lastPostion){
        case Orienation:
          roation = 0;
          exnteion = 15;
          lastPostion = SequenceType.Orienation;
          break;
        
        case GridFloor:
          roation = 0;
          exnteion = 15;
          lastPostion = SequenceType.GridFloor;
          break;

        case GridLevel1:
          roation = 30;
          exnteion = 15;
          lastPostion = SequenceType.GridLevel1;
          break;

        case GridLevel2:
          roation = 107;
          exnteion = 5;
          lastPostion = SequenceType.GridLevel2;
          break;
      }
    }

    addCommands
    (new extendArmToLength(0),
    new RotateArmToPoint(roation),
    new extendArmToLength(exnteion));
  }
}
