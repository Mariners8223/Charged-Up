// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import javax.xml.crypto.KeySelector.Purpose;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.SequenceType;

public class LimeLight extends SubsystemBase {
  private static LimeLight instsance;
  private static NetworkTable table;
  private static double hasTargets;
  private SequenceType sequenceType;
  private double distanceToTarget;
  /** Creates a new LimeLight. */
  private LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight-marines");
    hasTargets = 0;
    sequenceType = SequenceType.Reflective_Tape;
    distanceToTarget = 0;
  }

  public static LimeLight getInstance(){
    if(instsance == null){
      instsance = new LimeLight();
    }
    return instsance;
  }

  public void SetLimeLightMode(SequenceType sequenceType){
    this.sequenceType = sequenceType;
    switch(sequenceType){
      case Reflective_Tape:
        if(table.getEntry("getpipe").getDouble(0.0) != 0){
          table.getEntry("pipeline").setDouble(0);
          break;
        }
      
      case April_Tags:
        if(table.getEntry("getpiple").getDouble(0.0) != 1){
          table.getEntry("pipeline").setDouble(1);
          break;
        }

      default:
      table.getEntry("pipeline").setDouble(0);
      break;
    }
      
  }

  @Override
  public void periodic() {
    hasTargets = table.getEntry("tv").getDouble(0.0);
    // double distance = 55  -95/ Math.tan((-5 + table.getEntry("ty").getDouble(0.0)));
    // System.out.println(table.getEntry("ty").getDouble(0.0));
    // d = (h1-h2)/tan(a1+a2)
    

    if(hasTargets == 1){
      switch(sequenceType){
        case Reflective_Tape:
        double conehight;
          if(table.getEntry("ty").getDouble(0.0) < 0){
            conehight = 60; //h2 //5
          }
          else{
            conehight = 115;
          }
          
          double limelighthight = 95; //h1 // 99
          double limelightoffset = -3.5; //a1
          distanceToTarget = Math.abs((limelighthight - conehight)/Math.tan(Units.degreesToRadians(limelightoffset + 
          table.getEntry("ty").getDouble(0.0))));

          double angle = table.getEntry("tx").getDouble(0.0);
          double cosOfAngle = Math.cos(angle);
          double distenceX = cosOfAngle * distanceToTarget;
          SmartDashboard.putNumber("distX", distenceX);
          SmartDashboard.putNumber("dist", distanceToTarget);
      }
    }


    //upper cone is 112 cm and lower cone is 60
    //offset is -5 dgrees
    // limelight hight is 95




    // This method will be called once per scheduler run
  }
}
