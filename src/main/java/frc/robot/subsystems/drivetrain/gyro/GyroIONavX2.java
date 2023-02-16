package frc.robot.subsystems.drivetrain.gyro;

import com.kauailabs.navx.frc.AHRS;


public class GyroIONavX2 implements GyroIO {
  private AHRS gyro;
  private static GyroIONavX2 instance;

  private GyroIONavX2() {
    gyro = new AHRS();
    calibrateNavX();
  }

  public static GyroIONavX2 getInstance() {
    if (instance == null) 
      instance = new GyroIONavX2();
    return instance;
  }


  public void calibrateNavX() { gyro.calibrate(); }

  /** Resets the Yaw (Z Axis) of the gyro */
  public void resertNavX() { gyro.reset(); }

  public boolean isConnected() { return gyro.isConnected(); }

  public double getAngleDeg() { return gyro.getAngle(); }

  public double getPitch() { return gyro.getPitch(); }

  public double getAngleRateDeg() { return gyro.getRate(); }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.positionDeg = getAngleDeg();
    inputs.robotPitch = getPitch();
    inputs.isConnected = gyro.isConnected();
  }

}
