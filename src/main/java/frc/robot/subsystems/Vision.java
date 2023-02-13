// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {
  private static Vision instance;
  private final Field2d m_field = new Field2d();
  private static Pose3d pose3d;

  /** Creates a new PhotonVision. */
  private PhotonCamera camera = new PhotonCamera("mariners-cam");
  private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  private Pose2d pose2d = Drive.getInstance().getPose();

  public static Vision GetInstance(){
    if(instance == null){
      instance = new Vision();
    }
    return instance;
  }
  AprilTagFieldLayout aprilTagFieldLayout;
  
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
  
   
  private Vision() {
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    SmartDashboard.putData("Field", m_field);
  }

public Pose2d getpose2d(){
  return pose2d;
}
public Pose3d getPose3d(){
  return pose3d;
}

  @Override
  public void periodic() {
    var result = camera.getLatestResult();
    result = camera.getLatestResult();
    if(result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();
      int TargetID = target.getFiducialId();
      double Pose = target.getPoseAmbiguity();
      Transform3d trans = target.getBestCameraToTarget();
      SmartDashboard.putBoolean("Has target", result.hasTargets());
      SmartDashboard.putNumber("ID", TargetID);
      SmartDashboard.putNumber("ambgouitu", Pose);
      Optional<EstimatedRobotPose> x = getEstimatedGlobalPose(pose2d);
      EstimatedRobotPose camPose = x.get();
      pose3d = camPose.estimatedPose;
      pose2d = camPose.estimatedPose.toPose2d();
      m_field.setRobotPose(pose2d);
    }
    else{
      SmartDashboard.putBoolean("Has target", result.hasTargets());
      SmartDashboard.putNumber("ID", 0);
      SmartDashboard.putNumber("ambgouitu", 0);
    }
  }


  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d pose2d2) {
    return null;
  }
}
