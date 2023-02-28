package frc.robot.subsystems.orientation;

import javax.swing.text.html.HTMLDocument.HTMLReader.PreAction;

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
  private VictorSPX orientationUpMotor;
  private VictorSPX orientationDownMotor;
  private DoubleSolenoid orientationRampDoubleSolenoid;
  private DoubleSolenoid orientationUpDoubleSolenoid;
  public boolean isRunning; // I'm too lazy to implement an efficient system someone else do it

  private OrientationIOVictorSPX() {
    orientationUpMotor = new VictorSPX(RobotConstants.ORIENTATION_UP_MOTOR);
    orientationDownMotor = new VictorSPX(RobotConstants.ORIENTATION_DOWN_MOTOR);
    orientationRampDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[0], RobotConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[1]);
    orientationRampDoubleSolenoid.set(Value.kReverse);
    orientationUpDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[2], RobotConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[3]);
    orientationUpDoubleSolenoid.set(Value.kReverse);
    isRunning = false;
  }

  public static OrientationIOVictorSPX getInstance() {
    if (instance == null)
      instance = new OrientationIOVictorSPX();
    return instance;
  }

  @Override
  public void updateInputs(OrientationIOInputs inputs) {
    inputs.isClosed = (Value.kReverse == orientationRampDoubleSolenoid.get());
    inputs.isRunning = isRunning;
  }

  @Override
  public void toggleRampSolenoid() {
    orientationRampDoubleSolenoid.toggle();
  }

  public void toggleUpSolenoid(){
    orientationUpDoubleSolenoid.toggle();
  }

  public void disableMotors(){
    orientationDownMotor.set(ControlMode.Disabled, 0);
    orientationUpMotor.set(ControlMode.Disabled, 0);
  }
  
  @Override
  public void setPercent(double percent) {
    if(percent != 0){orientationUpMotor.set(ControlMode.PercentOutput, -percent- 0.2);}
    orientationDownMotor.set(ControlMode.PercentOutput, percent);
  }

}
