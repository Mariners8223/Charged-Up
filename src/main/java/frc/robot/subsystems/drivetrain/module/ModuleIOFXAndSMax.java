package frc.robot.subsystems.drivetrain.module;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.PIDFGains;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;

public class ModuleIOFXAndSMax implements ModuleIO {
  private CANSparkMax steeringMotor;
  private SparkMaxPIDController steeringPIDController;
  private RelativeEncoder steeringEncoder;

  private CANCoder absEncoder;

  private TalonFX driveMotor;

  private double steeringSetpoint;
  private double driveSetpoint;

  public ModuleIOFXAndSMax(SwerveModuleConstants module) {
    driveMotor = new TalonFX(module.idDrive);
    driveMotor.config_kP(0, module.driveGains.getP());
    driveMotor.config_kI(0, module.driveGains.getI());
    driveMotor.config_kI(0, module.driveGains.getD());
    driveMotor.config_kF(0, module.driveGains.getF());
    driveMotor.config_IntegralZone(0, module.driveGains.getIZone());

    steeringMotor = configSparkMax(module.idSteering, steeringPIDController, steeringEncoder, module.steeringGains);
    steeringPIDController = steeringMotor.getPIDController();
    steeringEncoder = steeringMotor.getEncoder();

    setPIDGains(steeringPIDController, module.steeringGains);

    steeringSetpoint = 0;

    absEncoder = configCANCoder(module.canCoderId, module.cancoderZeroAngle);
    calibrateSteering();

  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(
        driveMotor.getSelectedSensorPosition() / Drivetrain.SwerveModuleConstants.driveRatio
            / Constants.FALCON500_COUNTS_PER_REVOLUTION);
    inputs.driveVelocityPerSecRad = Units.rotationsPerMinuteToRadiansPerSecond(
        driveMotor.getSelectedSensorVelocity() * 10 / Constants.FALCON500_COUNTS_PER_REVOLUTION
            / Drivetrain.SwerveModuleConstants.driveDPRMeters);
    inputs.steeringAbsolutePositionRad = Units.rotationsToRadians(getAbsSteeringPos());
    inputs.steeringPositionRad = Units.degreesToRadians(getAngle());
    inputs.steeringVelocityPerSecRad = Units.rotationsPerMinuteToRadiansPerSecond(
        steeringEncoder.getVelocity() / Drivetrain.SwerveModuleConstants.steeringRatio);
  }

  private static CANSparkMax configSparkMax(int id, SparkMaxPIDController pidController, RelativeEncoder encoder,
      PIDFGains gains) {
    CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
    sparkMax.setInverted(false);

    return sparkMax;
  }

  private static CANCoder configCANCoder(int id, double zeroAngle) {

    CANCoder canCoder = new CANCoder(id);
    // Always set CANCoder relative encoder to 0 on boot
    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    // Configure the offset angle of the magnet
    canCoder.configMagnetOffset(360 - zeroAngle);

    return canCoder;
  }

  private static void setPIDGains(SparkMaxPIDController pidController, PIDFGains gains) {
    pidController.setI(gains.getI());
    pidController.setP(gains.getP());
    pidController.setD(gains.getD());
    pidController.setFF(gains.getF());
    pidController.setIZone(gains.getIZone());
    pidController.setOutputRange(-1.0, 1.0);
  }

  public void calibrateSteering() {
    this.steeringEncoder
        .setPosition(absEncoder.getAbsolutePosition() / 360 / Drivetrain.SwerveModuleConstants.steeringRatio);
  }

  public void stop() {
    driveMotor.set(ControlMode.Disabled, 0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = optimizeAngle(desiredState, Rotation2d.fromDegrees(getAngle()));

    steeringSetpoint = addDeltaFromZeroToEncoder(state.angle.getDegrees());
    driveSetpoint = driveVelocityToRPM(state.speedMetersPerSecond);

    if (state.speedMetersPerSecond != 0) {
      steeringPIDController.setReference(steeringSetpoint, ControlType.kPosition);
    }
    // _driveSparkMax.set(1 * Math.signum(state.speedMetersPerSecond));
    if (state.speedMetersPerSecond == 0)
      driveMotor.set(ControlMode.Disabled, 0);
    else
      driveMotor.set(ControlMode.Velocity, driveSetpoint);
  }

  public static SwerveModuleState optimizeAngle(SwerveModuleState desiredState, Rotation2d currentRadian) {
    Rotation2d angle = desiredState.angle.minus(currentRadian);
    double speed = desiredState.speedMetersPerSecond;
    if (Math.abs(angle.getDegrees()) > 90) {
      speed = -speed;
      if (angle.getRadians() > 0) {
        angle = angle.minus(Rotation2d.fromDegrees(180));
      } else {
        angle = angle.plus(Rotation2d.fromDegrees(180));
      }
    }
    return new SwerveModuleState(speed, angle);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Units.rotationsPerMinuteToRadiansPerSecond(
            driveMotor.getSelectedSensorVelocity() * 10 / Drivetrain.SwerveModuleConstants.driveRatio)
            * Drivetrain.SwerveModuleConstants.driveDPRMeters,
        new Rotation2d()
            .rotateBy(Rotation2d.fromDegrees(Units.rotationsToDegrees(steeringEncoder.getPosition()))));
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        driveMotor.getSelectedSensorPosition() / Constants.FALCON500_COUNTS_PER_REVOLUTION
            / Drivetrain.SwerveModuleConstants.driveRatio,
        new Rotation2d().rotateBy(Rotation2d.fromDegrees(Units.rotationsToDegrees(
            steeringEncoder.getPosition() / Drivetrain.SwerveModuleConstants.steeringRatio))));
  }

  public double addDeltaFromZeroToEncoder(double angle) {
    double pos = getAbsSteeringPos();
    return (pos + (angle / 360)) / Drivetrain.SwerveModuleConstants.steeringRatio;
  }

  private double driveVelocityToRPM(double velocity) {
    // divide by distance per revolution, multiply by a minute to get RPM
    return velocity / (Drivetrain.SwerveModuleConstants.driveDPRMeters) * 60;
  }

  public double getAbsSteeringPos() {
    return steeringEncoder.getPosition() * Drivetrain.SwerveModuleConstants.steeringRatio;
  }

  public double getAngle() {
    double pos = getAbsSteeringPos();
    pos = pos - Math.floor(pos);
    return pos * 360;
  }

  public double getSteeringSetpoint() {
    return steeringSetpoint;
  }

  public double getDriveSetpoint() {
    return driveSetpoint;
  }

  public double getDriveRPM() {
    return driveMotor.getSelectedSensorVelocity();
  }

  public void setDriveRPM(double RPM) {
    driveMotor.set(ControlMode.Velocity, RPM);
  }

  public void resetSteeringEncoder() {
    steeringEncoder.setPosition(0);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setTurnBrakeMode(boolean enable) {
    steeringMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}