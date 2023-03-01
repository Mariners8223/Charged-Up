// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TempConstants;

public class Tank extends SubsystemBase {
  private static Tank instance;
  private static VictorSPX engine1;
  private static VictorSPX engine2;
  private static VictorSPX engine3;
  private static VictorSPX engine4;
  /** Creates a new Tank. */
  private Tank() {
    engine1 = new VictorSPX(TempConstants.ENGINE1_ID); // right leader
    engine2 = new VictorSPX(TempConstants.ENGINE2_ID); // right follower
    engine3 = new VictorSPX(TempConstants.ENGINE3_ID); // left leader
    engine4 = new VictorSPX(TempConstants.ENGINE4_ID); // left follower
    engine1 = new VictorSPX(TempConstants.ENGINE1_ID);
    engine2 = new VictorSPX(TempConstants.ENGINE2_ID);
    engine3 = new VictorSPX(TempConstants.ENGINE3_ID);
    engine4 = new VictorSPX(TempConstants.ENGINE4_ID);
    engine2.setInverted(true);
    engine4.setInverted(true);
    engine2.follow(engine1);
    engine4.follow(engine3);
  }
  public static Tank getinstance(){
    if(instance == null){
      instance = new Tank();
    }
    return instance;
  }

  public void setRightSpeed(double speed){
    engine1.set(ControlMode.PercentOutput, -speed);
  }

  public void setLeftSpeed(double speed){
    engine3.set(VictorSPXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
