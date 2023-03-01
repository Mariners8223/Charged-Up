// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TempConstants;

public class Tank extends SubsystemBase {
  private static Tank instance;
  private static MotorController engine1;
  private static MotorController engine2;
  private static MotorController engine3;
  private static MotorController engine4;
  private static MotorControllerGroup left;
  private static MotorControllerGroup right;
  private static DifferentialDrive drive;
  /** Creates a new Tank. */
  private Tank() {
    engine1 = new WPI_VictorSPX(TempConstants.ENGINE1_ID); // right leader
    engine2 = new WPI_VictorSPX(TempConstants.ENGINE2_ID); // right follower
    engine3 = new WPI_VictorSPX(TempConstants.ENGINE3_ID); // left leader
    engine4 = new WPI_VictorSPX(TempConstants.ENGINE4_ID); // left follower
    engine2.setInverted(false);
    engine4.setInverted(false);
    left = new MotorControllerGroup(engine3, engine4);
    right = new MotorControllerGroup(engine1, engine2);
    drive = new DifferentialDrive(left, right);
  }
  public static Tank getinstance(){
    if(instance == null){
      instance = new Tank();
    }
    return instance;
  }

  @Override
  public void periodic() {
    double X = RobotContainer.getRawAxis(0);
    double Y = RobotContainer.getRawAxis(1);
    Y = Y/1.5;
    drive.arcadeDrive(Y, X);
  }
}
