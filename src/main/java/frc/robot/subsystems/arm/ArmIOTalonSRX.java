package frc.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.drivetrain.Drivebase.controlMode;

public class ArmIOTalonSRX implements ArmIO {
  private TalonFX rotationMotor;
  private TalonSRX extensionMotor;
  private DigitalInput extensionLimitSwitch;
  private DigitalInput rotationLimitSwitch;

  private static ArmIOTalonSRX instance;
    
  private ArmIOTalonSRX() {
    rotationMotor = new TalonFX(ArmConstants.ARM_ROTATION);
    extensionMotor = new TalonSRX(ArmConstants.ARM_EXTENSION);
    extensionLimitSwitch = new DigitalInput(ArmConstants.EXTENSION_LIMIT_SWITCH_PORT);
    // rotationLimitSwitch = new DigitalInput(ArmConstants.ROTATION_LIMIT_SWITCH_PORT);

    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);


    rotationMotor.config_kP(0, ArmConstants.ARM_ROTATION_KP);
    rotationMotor.config_kI(0, ArmConstants.ARM_ROTATION_KI);
    rotationMotor.config_kD(0, ArmConstants.ARM_ROTATION_KD);
    rotationMotor.config_kF(0, ArmConstants.ARM_ROTATION_KF);
    rotationMotor.setInverted(InvertType.InvertMotorOutput);
    rotationMotor.configOpenloopRamp(0.2);
    rotationMotor.configClosedloopRamp(0.2);

    extensionMotor.config_kP(0, ArmConstants.ARM_EXTENSION_KP);
    extensionMotor.config_kI(0, ArmConstants.ARM_EXTENSION_KI);
    extensionMotor.config_kD(0, ArmConstants.ARM_EXTENSION_KD);
    extensionMotor.setSensorPhase(true);
    extensionMotor.setInverted(InvertType.None);

    
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.setNeutralMode(NeutralMode.Brake);


    

    rotationMotor.setSelectedSensorPosition(0);
    extensionMotor.setSelectedSensorPosition(0);
  }

  public static ArmIOTalonSRX getInstance() {
    if (instance == null)
      instance = new ArmIOTalonSRX();
    return instance;
  }

  public void resetExtensionEncoder(double CM) {
    extensionMotor.setSelectedSensorPosition(CM * ArmConstants.ARM_EXTENSION_CM_PER_REVOLUTION);
  }

  public void resetRotationEncoder(double CM){
    rotationMotor.setSelectedSensorPosition(CM* ArmConstants.ARM_ROTATION_GEAR_RATIO);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // inputs.armAngleDeg = getArmAngleDeg();
    // inputs.armExtensionMeters = getArmLengthMeters();
  }

  public void resetRotation(){
    rotationMotor.setSelectedSensorPosition(0);
  }

  public boolean getExtensionLimitSwitch() {
    return !extensionLimitSwitch.get();
  }

  public boolean getRotationLimitSwitch(){
    return !rotationLimitSwitch.get();
  }


  public double getRotationEncoder() {
    return rotationMotor.getSelectedSensorPosition();
  }

  public double getArmEncoder() {
    return extensionMotor.getSelectedSensorPosition();
  }

  public void resetExtension(){
    extensionMotor.setSelectedSensorPosition(0);
  }

  public double getClosedLoopRotationError() {
    return rotationMotor.getClosedLoopError();
  }
  
  public double getArmAngleDeg() {
    return rotationMotor.getSelectedSensorPosition() / ArmConstants.ARM_REVOLUTIONS_PER_DEGREE;
  }

  public double getExtensionOutputPercent() {
    return extensionMotor.getMotorOutputPercent();
  }


  public double getArmLengthMeters() {
    return extensionMotor.getSelectedSensorPosition() * ArmConstants.ARM_EXTENSION_CM_PER_REVOLUTION;
  }

  public boolean isArmAtSetpoint() {
    return Math.abs(rotationMotor.getSelectedSensorPosition() - rotationMotor.getClosedLoopTarget()) < ArmConstants.ARM_ROTATION_TOLERANCE;
  }
  public boolean isArmAtExtensionSetpoint() {
    return Math.abs(extensionMotor.getSelectedSensorPosition() - extensionMotor.getClosedLoopTarget()) < ArmConstants.ARM_EXTENSION_TOLERANCE;
  }
  public void stopRotation() {
    rotationMotor.set(ControlMode.Disabled, 0);
  }

  public void stopExtension() {
    extensionMotor.set(ControlMode.Disabled, 0);
  }
 
  @Override
  public void moveToAngle(double desiredAnglesDeg) {
    rotationMotor.set(ControlMode.Position, desiredAnglesDeg * ArmConstants.ARM_REVOLUTIONS_PER_DEGREE);
  }  

  @Override
  public void extendToLength(double extensionCM) {
    extensionMotor.set(ControlMode.Position, extensionCM * ArmConstants.ARM_EXTENSION_CM_PER_REVOLUTION);
  }

  public void setExtensionPrecent(double speed){
    extensionMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setRotationPrecent(double speed){
    rotationMotor.set(ControlMode.PercentOutput, speed);
  }

}
