// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  private static LimeLight Instance;
  private static double offsetX;
  private static double offsetY;
  private static double latancy;
  /** Creates a new LimeLight. */
  public static LimeLight getInstance(){
    if(Instance == null){ Instance = new LimeLight();}
    return Instance;
  }
  private LimeLight() {
   
  }

  @Override
  public void periodic() {
    if( NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
      offsetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      offsetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      latancy =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0) + 11;
    }
  }
}
