package frc.robot.subsystems.orientation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.OrientationConstants;
import frc.robot.Constants.RobotConstants;

public class OrientationIOVictorSPX implements OrientationIO {
  private static OrientationIOVictorSPX instance;
  private VictorSPX orientationLeftMotor;
  private VictorSPX orientationRightMotor;
  private DoubleSolenoid orientationDoubleSolenoid;
  public boolean isRunning; // I'm too lazy to implement an efficient system someone else do it

  private OrientationIOVictorSPX() {
    orientationLeftMotor = new VictorSPX(RobotConstants.ORIENTATION_LEFT_MOTOR);
    orientationRightMotor = new VictorSPX(RobotConstants.ORIENTATION_RIGHT_MOTOR);
    orientationLeftMotor.setInverted(true);
    orientationRightMotor.setInverted(false);
    orientationDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, OrientationConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[0], OrientationConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[1]);
    orientationDoubleSolenoid.set(Value.kReverse);
    isRunning = false;
  }

  public static OrientationIOVictorSPX getInstance() {
    if (instance == null)
      instance = new OrientationIOVictorSPX();
    return instance;
  }

  @Override
  public void updateInputs(OrientationIOInputs inputs) {
    inputs.isClosed = (Value.kReverse == orientationDoubleSolenoid.get());
    inputs.isRunning = isRunning;
  }

  @Override
  public void toggleOrientation() {
    orientationDoubleSolenoid.toggle();
  }
  
  @Override
  public void setPercent(double percenttop, double percentbottom) {
    orientationLeftMotor.set(ControlMode.PercentOutput, percenttop);
    orientationRightMotor.set(ControlMode.PercentOutput, percentbottom);
  }

}
