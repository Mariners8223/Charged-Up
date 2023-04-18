// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private static Arm instance;
  private final ArmIOTalonSRX io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private boolean extensionCalibrated;
  private boolean rotationCalibrated;
  

  private Arm(ArmIOTalonSRX io) {
    this.io = io;
    new Trigger(this::getExtensionLimitSwitch).onTrue(new PrintCommand("fucking finally"));
    extensionCalibrated = false;
    rotationCalibrated = false;
  }

  public static Arm getInstance() {
    if (instance == null)
      instance = new Arm(ArmIOTalonSRX.getInstance());
    return instance;
  }

  public boolean isAtRotationSetpoint() {
    return io.isArmAtSetpoint();
  }
  public boolean isAtExtensionSetpoint(){
    return io.isArmAtExtensionSetpoint();
  }

  public void stopRotationMotor() {
    io.stopRotation();
  }

  public void stopExtensionMotor() {
    io.stopExtension();
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);
    SmartDashboard.putNumber("rotation sensor position", io.getArmAngleDeg());
    SmartDashboard.putNumber("Extension Length CM", io.getArmEncoder() / ArmConstants.ARM_EXTENSION_CM_PER_REVOLUTION);
    SmartDashboard.putBoolean("limit switch", io.getExtensionLimitSwitch());
    if (io.getExtensionLimitSwitch()) io.resetExtensionEncoder(-2);
  }

  public void extendToLengthMeters(double lengthMeters) {
    io.extendToLength(lengthMeters);
  }

  public void rotateToAngleDegrees(double desiredAngleDeg) {
    io.moveToAngle(desiredAngleDeg);
  }

  public Boolean getExtensionLimitSwitch(){ return io.getExtensionLimitSwitch(); }

  public boolean getRotationLimitSwitch(){ return io.getRotationLimitSwitch();}

  public void resetRotationEncoder(double CM){
    io.resetExtensionEncoder(CM);
  }

  public void setFalconPO(double speed) {
    if(io.getArmAngleDeg() >= -20 && io.getArmAngleDeg() <= 65){ speed = 0;}
    io.setRotationPrecent(speed);
  }

  public void resetExtensionEncoder(double CM) {
    io.resetExtensionEncoder(CM);
  }
  public void set775PO(double speed) {
    if(io.getArmAngleDeg() >= -20 && io.getArmAngleDeg() <= 65 && Arm.getInstance().isExtensionCalibrated()){ speed = 0;}
    io.setExtensionPrecent(speed);
  }

  public void setExtenstionCalibrated(boolean isCalibrated){
    extensionCalibrated = isCalibrated;
  }

  public void setRotationCalibrated(boolean isCalibrated){
    rotationCalibrated = isCalibrated;
  }

  public boolean isExtensionCalibrated() {
    return extensionCalibrated;
  }

  public boolean isRotationCalibrated(){
    return rotationCalibrated;
  }

  public void updateSetpoint(ArmSetpoint setpoint) {
    SmartDashboard.putString("CURRENT POSITION", setpoint.toString());
  }

  public static enum ArmSetpoint {
    HomePosition(0),
    GridLow(1),
    GridMid(2),
    GridHigh(3),
    DoubleSubstation(4);

    public final int value;
    ArmSetpoint(int index) {
      this.value = index;
    }

    @Override
    public String toString() {
      var position = this.name();
      return "" + position;
    }
  }

}
