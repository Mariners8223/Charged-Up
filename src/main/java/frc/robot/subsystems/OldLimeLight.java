// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OldLimeLight extends SubsystemBase {
  //creates all the vars
  private static OldLimeLight Instance;
  private static PhotonCamera LimeLight;
  private static Double distanceToTarget;
  private static double YawToTarget;
  private static double timeStamp;
  private static boolean isLimeLightModeAprilTags;
  private static double latency;
  
  //creates new instance
  public static OldLimeLight getInstance(){
    if(Instance == null){ Instance = new OldLimeLight();}
    return Instance;
  }

  //creates the constructor
  private OldLimeLight() {
    LimeLight = new PhotonCamera("limelight-mariners");
    isLimeLightModeAprilTags = true;
    latency = 0.0;
    distanceToTarget = 0.0;
    YawToTarget = 0.0;
    timeStamp = 0.0;

    SmartDashboard.putNumber("distance to target", distanceToTarget);
    SmartDashboard.putNumber("pitch to target", YawToTarget);
    SmartDashboard.putBoolean("limeLightAprilTagMode", isLimeLightModeAprilTags);
  }

  public boolean getIsLimeLightModeAprilTags(){
    return isLimeLightModeAprilTags;
  }

  public double getDistanceToTarget(){
    return distanceToTarget;
  }
  public double getYawToTarget(){
    return YawToTarget;
  }

  public double getLatency(){
    return latency;
  }

  public double getTimeStamp(){
    return timeStamp;
  }

  public void setIsLimeLightModeAprilTags(boolean mode){
    isLimeLightModeAprilTags = mode;
    SmartDashboard.putBoolean("LimeLightModeAprilTags", mode);
  }

  //preiodic shit
  @Override
  public void periodic() {
    isLimeLightModeAprilTags = SmartDashboard.getBoolean("limeLightAprilTagMode", isLimeLightModeAprilTags);
    Logger.getInstance().recordOutput("LimeLight/IsLimeLightModeAprilTags", isLimeLightModeAprilTags);
    if(isLimeLightModeAprilTags){
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
    YawToTarget = 0.0;
    timeStamp = 0.0;
    if(result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();
      distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(Constants.robotToLimeLight.getZ(), target.getBestCameraToTarget().getZ(), Math.toRadians(Constants.robotToLimeLight.getRotation().getAngle()), Math.toRadians(target.getPitch()));
      YawToTarget = target.getYaw();
      timeStamp = result.getTimestampSeconds();
      latency = result.getLatencyMillis();
      SmartDashboard.putNumber("distance to target", distanceToTarget);
      SmartDashboard.putNumber("pitch to target", YawToTarget);
      Logger.getInstance().recordOutput("LimeLight/PitchToTarget", YawToTarget);
      Logger.getInstance().recordOutput("LimeLight/DistanceToTarget", distanceToTarget);
      Logger.getInstance().recordOutput("LimeLight/Latency", latency);
    }
  }
}
