// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.ejml.data.CMatrixRMaj;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
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
  private static Double cameraToTarget;
  
  //creates new instance
  public static LimeLight getInstance(){
    if(Instance == null){ Instance = new LimeLight();}
    return Instance;
  }

  //creates the constructor
  private LimeLight() {
    LimeLight = new PhotonCamera("limelight-mariners");
    SmartDashboard.putBoolean("tape", false);

  }



  //preiodic shit
  @Override
  public void periodic() {
    if(SmartDashboard.getBoolean("tape", false)){
      LimeLight.setPipelineIndex(2);
      LimeLight.setLED(VisionLEDMode.kOn);
    }
    else{
      LimeLight.setPipelineIndex(1);
      LimeLight.setLED(VisionLEDMode.kOff);
    }
    var result = LimeLight.getLatestResult();
    if(result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();
      cameraToTarget = PhotonUtils.calculateDistanceToTargetMeters(Constants.robotToLimeLight.getZ(), target.getBestCameraToTarget().getZ(), 0, target.getYaw());
    }
  }
}
