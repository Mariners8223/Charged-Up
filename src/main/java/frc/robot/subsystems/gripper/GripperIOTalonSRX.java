package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.RobotConstants;

public class GripperIOTalonSRX implements GripperIO {
  private TalonSRX gripperMotor;
  private static GripperIOTalonSRX instance;

  private GripperIOTalonSRX() {
    gripperMotor = new TalonSRX(RobotConstants.GRIPPER_MOTOR);

    gripperMotor.config_kP(0, GripperConstants.GRIPPER_KP);
    gripperMotor.config_kI(0, 0);
    gripperMotor.config_kD(0, 0);
    gripperMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    gripperMotor.setNeutralMode(NeutralMode.Brake);
    gripperMotor.configPeakOutputForward(GripperConstants.GRIPPER_SPEED, 0);

    gripperMotor.setSelectedSensorPosition(0);
  }

  public static GripperIOTalonSRX getInstance() {
    if (instance == null)
      instance = new GripperIOTalonSRX();
    return instance;
  }

  public double getGripperRotationDegrees() {
    return Units.rotationsToDegrees(
      gripperMotor.getSelectedSensorPosition() / Constants.SRX_MAG_COUNTS_PER_REVOLUTION / GripperConstants.GRIPPER_GEAR_RATIO);
  }

  public double getGripperRotationDegreesPerSec() {
    return Units.radiansToDegrees(Units.rotationsPerMinuteToRadiansPerSecond(
      gripperMotor.getSelectedSensorVelocity() * 10 / Constants.SRX_MAG_COUNTS_PER_REVOLUTION / GripperConstants.GRIPPER_GEAR_RATIO));
  }


  @Override
  public void updateInputs(GripperIOInputs inputs) {
    inputs.gripperDeg = this.getGripperRotationDegrees();
    inputs.velocityPerSecDeg = this.getGripperRotationDegreesPerSec();
    inputs.isClosed = !(inputs.gripperDeg <= 5 || inputs.gripperDeg >= -5); 
  }
  
  @Override
  public void setVoltage(double voltage) {
    gripperMotor.set(ControlMode.PercentOutput, voltage);
  }

}
