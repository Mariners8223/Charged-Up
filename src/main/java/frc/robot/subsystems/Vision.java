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
import org.photonvision.targeting.PhotonPipelineResult;
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
  private static Field2d m_field;
  private static Vision instance;
  private static Pose3d pose3d;
  private static Pose2d pose2d;    
  private static Pose2d lastPose2d;
  private static PhotonCamera rasberryPiCamera;
  private static PhotonCamera limeLightCamera;
  private static PhotonPoseEstimator cameraPoseEstimator;
  private static PhotonPoseEstimator limeligPoseEstimator;
  private static double timeStamp;
  private static double latency;


  
  //creates new single tone
  public static Vision GetInstance(){
    if(instance == null){
      instance = new Vision();
    }
    return instance;
  }
  
  //creates new constractur
  private Vision() {
    pose2d = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    lastPose2d = pose2d;
    pose3d = new Pose3d(pose2d);
    timeStamp = 0;
    latency = 0.0;

    rasberryPiCamera = new PhotonCamera("mariners-cam");
    limeLightCamera = new PhotonCamera("limelight-mariners");//lime light camera?
    
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {}
    m_field = new Field2d();

    cameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, rasberryPiCamera, Constants.robotToCam);
    limeligPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, limeLightCamera, Constants.robotToLimeLight);
  
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("april tag id", 0);
    SmartDashboard.putNumber("latency", 0);
  }

  //returns a pose2d (no shit)
  public Pose2d getpose2d(){
    return pose2d;
  }

  //returns a pose3d
  public Pose3d getPose3d(){
    return pose3d;
  }

  //returns timestamp
  public double GetTimestamp(){
    return timeStamp;
  }

  public double getLatency(){
    return latency;
  }

  //creats the new pose for camera
  private Optional<EstimatedRobotPose> getEstimatedPose(Pose2d lastPose2d, PhotonPoseEstimator bestPoseEstimator) {
    bestPoseEstimator.setReferencePose(lastPose2d);
    return bestPoseEstimator.update();
  }

  //creates the new pose for limelight

  @Override
  public void periodic() {
    var resultRasberryPiCamera = rasberryPiCamera.getLatestResult();
    var resultLimelight = limeLightCamera.getLatestResult();
    PhotonTrackedTarget target = null;
    PhotonPipelineResult bestResult = null;
    PhotonPoseEstimator bestPoseEstimator = null;

    if(!resultRasberryPiCamera.hasTargets() && !(resultLimelight.hasTargets() &&  LimeLight.getInstance().getIsLimeLightModeAprilTags())){
      pose2d = null;
      pose3d = null;
      timeStamp = 0;
      return;
    }


    double cameraAMB = 100;
    double limelightAMB = 100;
    if(resultRasberryPiCamera.hasTargets()){
      cameraAMB = resultRasberryPiCamera.getBestTarget().getPoseAmbiguity();
    }
    if(resultLimelight.hasTargets() && LimeLight.getInstance().getIsLimeLightModeAprilTags()){
      limelightAMB = resultLimelight.getBestTarget().getPoseAmbiguity();
    }

    if(cameraAMB < limelightAMB){
      bestResult = resultRasberryPiCamera;
      bestPoseEstimator = cameraPoseEstimator;
      Logger.getInstance().recordOutput("cameraResultSource", "pi");
    }else {
      bestResult = resultLimelight;
      bestPoseEstimator = limeligPoseEstimator;
      Logger.getInstance().recordOutput("cameraResultSource", "limelight");
    }
    
    target = bestResult.getBestTarget();
    timeStamp = bestResult.getTimestampSeconds();
    latency = bestResult.getLatencyMillis();

    Optional<EstimatedRobotPose> eOptional = getEstimatedPose(lastPose2d, bestPoseEstimator);
    EstimatedRobotPose camPose = eOptional.get();
    pose3d = camPose.estimatedPose;
    lastPose2d = pose2d;
    pose2d = camPose.estimatedPose.toPose2d();
    m_field.setRobotPose(pose2d);


    SmartDashboard.putNumber("april tag id", target.getFiducialId());
    SmartDashboard.putNumber("latency", latency);
    SmartDashboard.putData(m_field);

    Logger.getInstance().recordOutput("pose3d", pose3d);
    Logger.getInstance().recordOutput("pose2d", pose2d);
    Logger.getInstance().recordOutput("poseAmbiguity", target.getPoseAmbiguity());
    Logger.getInstance().recordOutput("aprilTagId", target.getFiducialId());
    Logger.getInstance().recordOutput("latency", bestResult.getLatencyMillis());
    
  }
}
