package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotConstants;

public class ArmIOTalonSRX implements ArmIO {
  private TalonFX rotationMotor;
  private TalonSRX extensionMotor;

  private static ArmIOTalonSRX instance;
    
  private ArmIOTalonSRX() {
    rotationMotor = new TalonFX(RobotConstants.ARM_ROTATION_MOTOR);
    extensionMotor = new TalonSRX(RobotConstants.ARM_EXTENSION_MOTOR);

    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    rotationMotor.config_kP(0, ArmConstants.ARM_ROTATION_KP);
    rotationMotor.config_kI(0, ArmConstants.ARM_ROTATION_KI);
    rotationMotor.config_kD(0, ArmConstants.ARM_ROTATION_KD);
    rotationMotor.config_kF(0, ArmConstants.ARM_ROTATION_KF);
    rotationMotor.setInverted(InvertType.InvertMotorOutput);

    extensionMotor.config_kP(0, ArmConstants.ARM_EXTENSION_KP);
    extensionMotor.config_kI(0, ArmConstants.ARM_EXTENSION_KI);
    extensionMotor.config_kD(0, ArmConstants.ARM_EXTENSION_KD);

    
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    rotationMotor.setSelectedSensorPosition(0);
    extensionMotor.setSelectedSensorPosition(0);
    rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
  }

  public static ArmIOTalonSRX getInstance() {
    if (instance == null)
      instance = new ArmIOTalonSRX();
    return instance;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAngleDeg = getArmAngleDeg();
    inputs.armExtensionMeters = getArmLengthMeters();
  }

  public void resetRotation(){
    rotationMotor.setSelectedSensorPosition(0);
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
    return extensionMotor.getSelectedSensorPosition() / Constants.SRX_MAG_COUNTS_PER_REVOLUTION * ArmConstants.DISTANCE_PER_REVOLUTION_CM;

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
    extensionMotor.set(ControlMode.Position, extensionCM / ArmConstants.DISTANCE_PER_REVOLUTION_CM * Constants.SRX_MAG_COUNTS_PER_REVOLUTION);
  }

  public void setExtensionPrecent(double speed){
    extensionMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setRotationPrecent(double speed){
    rotationMotor.set(ControlMode.PercentOutput, speed);
  }

}
