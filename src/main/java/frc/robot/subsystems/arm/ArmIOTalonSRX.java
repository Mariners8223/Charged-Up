package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotConstants;

public class ArmIOTalonSRX implements ArmIO {
  private TalonSRX rotationMotor;
  private TalonSRX extensionMotor;

  private static ArmIOTalonSRX instance;
    
  private ArmIOTalonSRX() {
    rotationMotor = new TalonSRX(RobotConstants.ARM_ROTATION_MOTOR);
    extensionMotor = new TalonSRX(RobotConstants.ARM_EXTENSION_MOTOR);

    rotationMotor.config_kP(0, ArmConstants.ARM_ROTATION_KP);
    rotationMotor.config_kI(0, ArmConstants.ARM_ROTATION_KI);
    rotationMotor.config_kD(0, ArmConstants.ARM_ROTATION_KD);
    rotationMotor.config_kF(0, ArmConstants.ARM_ROTATION_KF);

    extensionMotor.config_kP(0, ArmConstants.ARM_EXTENSION_KP);
    extensionMotor.config_kI(0, ArmConstants.ARM_EXTENSION_KI);
    extensionMotor.config_kD(0, ArmConstants.ARM_EXTENSION_KD);
    
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
  }

  public static ArmIOTalonSRX getInstance() {
    if (instance == null)
      instance = new ArmIOTalonSRX();
    return instance;
  }

  @Override
  public void setVoltage(double voltsRotation, double voltsExtension) {
  }
  
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAngleRad = getArmAngleRad();
    inputs.armVelocityPerSecRad = getArmVelocityPerSecRad();
    inputs.armExtensionMeters = getArmLengthMeters();
  }
  
  public double getArmAngleRad() {
    return Units.rotationsToRadians(
      rotationMotor.getSelectedSensorPosition() / Constants.SRX_MAG_COUNTS_PER_REVOLUTION / ArmConstants.ARM_ROTATION_GEAR_RATIO
    );
  }

  public double getArmVelocityPerSecRad() {
    return Units.rotationsPerMinuteToRadiansPerSecond(
      rotationMotor.getSelectedSensorVelocity() * 10 / Constants.SRX_MAG_COUNTS_PER_REVOLUTION / ArmConstants.ARM_ROTATION_GEAR_RATIO
    );
  }

  public double getArmLengthMeters() {
    return Units.inchesToMeters((extensionMotor.getSelectedSensorPosition() / Constants.SRX_MAG_COUNTS_PER_REVOLUTION / ArmConstants.ARM_EXTENSION_GEAR_RATIO) * ArmConstants.DISTANCE_PER_REVOLUTION);
  }
 
  @Override
  public void moveToAngle(double desiredAnglesDeg) {
    rotationMotor.set(ControlMode.Position, Units.degreesToRotations(desiredAnglesDeg));
  }  

  @Override
  public void extendToLength(double extensionMeters) {
    rotationMotor.set(ControlMode.Position, 
    (Units.metersToInches(extensionMeters) * Constants.SRX_MAG_COUNTS_PER_REVOLUTION * ArmConstants.ARM_EXTENSION_GEAR_RATIO) / ArmConstants.DISTANCE_PER_REVOLUTION
    );
  }
}
