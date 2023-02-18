// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.ejml.data.CMatrixRMaj;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  //creates all the vars
  private static LimeLight Instance;
  private static PhotonCamera LimeLight;
  private static Double distanceToTarget;
  private static double yawToTarget;
  private static double pitchToTarget;
  private static double timeStamp;
  private static boolean isAprilTags;
  
  //creates new instance
  public static LimeLight getInstance(){
    if(Instance == null){ Instance = new LimeLight();}
    return Instance;
  }

  //creates the constructor
  private LimeLight() {
    LimeLight = new PhotonCamera("limelight-mariners");
    isAprilTags = true;
    distanceToTarget = 0.0;
    yawToTarget = 0.0;
    pitchToTarget = 0.0;
    timeStamp = 0.0;

    SmartDashboard.putNumber("distance to target", distanceToTarget);
    SmartDashboard.putNumber("pitch to target", pitchToTarget);
    SmartDashboard.putBoolean("limeLightAprilTagMode", isAprilTags);
  }

  public boolean isAprilTags(){
    return isAprilTags;
  }

  public double getDistanceToTarget(){
    return distanceToTarget;
  }

  public double getYawToTarget(){
    return yawToTarget;
  }

  public double getPitchToTarget(){
    return pitchToTarget;
  }

  public double getTimeStamp(){
    return timeStamp;
  }

  //preiodic shit
  @Override
  public void periodic() {
    isAprilTags = SmartDashboard.getBoolean("limeLightAprilTagMode", isAprilTags);
    if(isAprilTags){
      if(LimeLight.getPipelineIndex() != 0){
        LimeLight.setPipelineIndex(0);
      }
      return;
    }
    if(LimeLight.getPipelineIndex() != 1){
      LimeLight.setPipelineIndex(1);
    }

    PhotonPipelineResult result = LimeLight.getLatestResult();
    distanceToTarget = 0.0;
    yawToTarget = 0.0;
    pitchToTarget = 0.0;
    timeStamp = 0.0;
    if(result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();
      distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(Constants.robotToLimeLight.getZ(), target.getBestCameraToTarget().getZ(), Constants.robotToLimeLight.getRotation().getAngle(), target.getYaw());
      yawToTarget = target.getYaw();
      pitchToTarget = target.getPitch();
      timeStamp = result.getTimestampSeconds();
      SmartDashboard.putNumber("distance to target", distanceToTarget);
      SmartDashboard.putNumber("pitch to target", pitchToTarget);
    }
  }
}
