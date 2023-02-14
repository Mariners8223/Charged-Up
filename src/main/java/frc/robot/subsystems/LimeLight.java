// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.ejml.data.CMatrixRMaj;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  //creates all the vars
  private static LimeLight Instance;
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private final Field2d m_field = new Field2d();
  private static Vision instance;
  private static Pose3d pose3d;
  private static Pose2d pose2d;    
  private static Pose2d lastPose2d;
  private static Pose3d lastPose3d;
  private static PhotonCamera camera;
  private static PhotonPoseEstimator photonPoseEstimator;
  
  //creates new instance
  public static LimeLight getInstance(){
    if(Instance == null){ Instance = new LimeLight();}
    return Instance;
  }

  //creates the constructor
  private LimeLight() {
    pose2d = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    lastPose2d = pose2d;
    pose3d = new Pose3d(pose2d);
    lastPose3d = pose3d;
    camera = new PhotonCamera("limelight-mariners");
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {}
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.robotToLimeLight);
  }

  public Pose2d getPose2d(){
    if(camera.getLatestResult().hasTargets()){
      return pose2d;
    }
    else{
      return null;
    }
  }


  public Pose3d getPose3d(){
    if(camera.getLatestResult().hasTargets()){
      return pose3d;
    }
    else{
      return null;
    }
  }

  public double getTimeStamp(){
    if(camera.getLatestResult().hasTargets()){
      return camera.getLatestResult().getTimestampSeconds();
    }
    else{
      return 0;
    }
  }

  //preiodic shit
  @Override
  public void periodic() {
  }
}
