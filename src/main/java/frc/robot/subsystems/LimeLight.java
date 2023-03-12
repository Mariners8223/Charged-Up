// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;
import java.util.function.DoubleToIntFunction;

import javax.xml.crypto.KeySelector.Purpose;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private double distanceToTargetX;
  private Pose2d robotPose;
  private double latancy;
  /** Creates a new LimeLight. */
  private LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight-marines");
    hasTargets = 0;
    sequenceType = SequenceType.Reflective_Tape;
    distanceToTarget = 0;
    distanceToTargetX = 0;
    robotPose = new Pose2d();
    latancy = 0;
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
        if(table.getEntry("getpipe").getDouble(0.0) != 1){
          table.getEntry("pipeline").setDouble(1);
          break;
        }

      default:
      table.getEntry("pipeline").setDouble(0);
      break;
    }
      
  }

  public SequenceType getLimeLightMode(){
    switch((int)(table.getEntry("getpipe").getDouble(0.0))){
      case 0:
        return sequenceType.Reflective_Tape;
      
      case 1:
        return sequenceType.April_Tags;

      default:
        return sequenceType.Reflective_Tape;
    }
  }


  public Pose2d getPose2d(){
    return robotPose;
  }

  public double getLatancy(){
    return latancy;
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
        SequenceType gridType;
        double conehight;
          if(table.getEntry("ty").getDouble(0.0) < 0){
            conehight = 60; //h2 //5
            gridType = SequenceType.GridLevel1;
          }
          else{
            conehight = 115;
            gridType = SequenceType.GridLevel2;
          }
          
          double limelighthight = 95; //h1 // 99
          double limelightoffset = -3.5; //a1
          distanceToTarget = Math.abs((limelighthight - conehight)/Math.tan(Units.degreesToRadians(limelightoffset + 
          table.getEntry("ty").getDouble(0.0))));

          double angle = table.getEntry("tx").getDouble(0.0);
          double cosOfAngle = Math.sin(angle);
          distanceToTargetX = Math.abs(cosOfAngle * distanceToTarget);

          switch(gridType){
            case GridLevel1:
              SmartDashboard.putBoolean("Is Target Allinged", distanceToTarget < 80 && distanceToTargetX < 35);
              break;

            case GridLevel2:
              SmartDashboard.putBoolean("Is Target Allinged", distanceToTarget < 100 && distanceToTargetX < 35);
              break;

          }

          SmartDashboard.putNumber("Distance To Target", distanceToTarget);
          SmartDashboard.putNumber("Distance To Target X", distanceToTargetX);
          break;

          case April_Tags:
          double April_Tag_ID = table.getEntry("tid").getDouble(0.0);
          String allince;
          switch(DriverStation.getAlliance()){
            case Red:
              allince = "botpose_wpired";
              break;

            case Blue:
              allince = "botpose_wpiblue";
              break;

            default:
              allince = "shit";
              break;
          }

          double[] robotPoseInDoubleArr = table.getEntry(allince).getDoubleArray(new double[6]);
          latancy = robotPoseInDoubleArr[6];
          robotPose = new Pose2d(
            new Translation3d(robotPoseInDoubleArr[0], robotPoseInDoubleArr[1], robotPoseInDoubleArr[2]).toTranslation2d(),
            new Rotation3d(robotPoseInDoubleArr[3], robotPoseInDoubleArr[4], robotPoseInDoubleArr[5]).toRotation2d()
          );

              
        }
    }

      
  }


    //upper cone is 112 cm and lower cone is 60
    //offset is -5 dgrees
    // limelight hight is 95




    // This method will be called once per scheduler run
 }

