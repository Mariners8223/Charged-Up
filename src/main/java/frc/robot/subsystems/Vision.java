// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  //creates all vars needed for here
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private final Field2d m_field = new Field2d();
  private static Vision instance;
  private static Pose3d pose3d;
  private static Pose2d pose2d;    
  private static Pose2d lastPose2d;
  private static Pose3d lastPose3d;
  private static PhotonCamera camera;
  private static PhotonCamera limeligt;
  private static PhotonPoseEstimator photonPoseEstimator;
  private static PhotonTrackedTarget target;
  
  //creates new single tone
  public static Vision GetInstance(){
    if(instance == null){
      instance = new Vision();
    }
    return instance;
  }
  
  //creates new constractur
  private Vision() {
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("april tag id", 0);
    SmartDashboard.putNumber("target pose ambiguity", 0);
    SmartDashboard.putNumber("latency", 0);
    pose2d = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    lastPose2d = pose2d;
    pose3d = new Pose3d(pose2d);
    lastPose3d = pose3d;
    camera = new PhotonCamera("mariners-cam");
    limeligt = new PhotonCamera("limelight-mariners");
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {}
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Constants.robotToCam);
  }

  //returns a pose2d
  public Pose2d getpose2d(){
    if(camera.getLatestResult().hasTargets()){
      return pose2d;
    }
    else{
      return null;
    }
  }

  //returns a pose3d
  public Pose3d getPose3d(){
    if(camera.getLatestResult().hasTargets()){
      return pose3d;
    }
    else{
      return null;
    }
  }

  //returns timestamp
  public double GetTimestamp(){
    if(camera.getLatestResult().hasTargets()){
      return camera.getLatestResult().getTimestampSeconds();
    }
    else{
      return 0;
    }
  }

  //creats the new pose;
  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d lastPose2d) {
    photonPoseEstimator.setReferencePose(lastPose2d);
    return photonPoseEstimator.update();
  }

  @Override
  public void periodic() {
    var resultcamera = camera.getLatestResult();
    var resultLimeLight = limeligt.getLatestResult();
    var result = resultcamera;
    if(resultcamera.hasTargets()){
      if(Math.min(resultcamera.getBestTarget().getPoseAmbiguity(), resultLimeLight.getBestTarget().getPoseAmbiguity()) == resultcamera.getBestTarget().getPoseAmbiguity()){
         target = resultcamera.getBestTarget();
         result = resultcamera;
      }else{
        target = resultLimeLight.getBestTarget();
        result = resultLimeLight;
      }
      
      SmartDashboard.putNumber("april tag id", target.getFiducialId());
      SmartDashboard.putNumber("target pose ambiguity", target.getPoseAmbiguity());
      SmartDashboard.putNumber("latency", result.getLatencyMillis());
      Optional<EstimatedRobotPose> eOptional = getEstimatedGlobalPose(pose2d);
      EstimatedRobotPose camPose = eOptional.get();
      lastPose3d = pose3d;
      pose3d = camPose.estimatedPose;
      lastPose2d = pose2d;
      pose2d = camPose.estimatedPose.toPose2d();
      m_field.setRobotPose(pose2d);
      SmartDashboard.putData(m_field);
      Logger.getInstance().recordOutput("2D Pose", pose2d);
    }
  }
}
