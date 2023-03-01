package frc.robot.subsystems.orientation;

import javax.swing.text.html.HTMLDocument.HTMLReader.PreAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.OrientationConstants;
import frc.robot.Constants.RobotConstants;

public class OrientationIOVictorSPX implements OrientationIO {
  private static OrientationIOVictorSPX instance;
  private VictorSPX orientationUpMotor;
  private VictorSPX orientationRampMotor;
  private DoubleSolenoid orientationRampDoubleSolenoid;
  private DoubleSolenoid orientationUpDoubleSolenoid;
  public boolean isRunning; // I'm too lazy to implement an efficient system someone else do it

  private OrientationIOVictorSPX() {
    orientationUpMotor = new VictorSPX(RobotConstants.ORIENTATION_UP_MOTOR);
    orientationRampMotor = new VictorSPX(RobotConstants.ORIENTATION_DOWN_MOTOR);
    orientationRampDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    orientationRampDoubleSolenoid.set(Value.kReverse);
    orientationUpDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[2], RobotConstants.ORIENTATION_DOUBLE_SOLENOID_PORTS[3]);
    orientationUpDoubleSolenoid.set(Value.kReverse);
    isRunning = false;
    orientationRampDoubleSolenoid.set(Value.kForward);
    orientationUpDoubleSolenoid.set(Value.kForward);
  }

  public static OrientationIOVictorSPX getInstance() {
    if (instance == null)
      instance = new OrientationIOVictorSPX();
    return instance;
  }

  @Override
  public void updateInputs(OrientationIOInputs inputs) {
    inputs.isOrientaionClosed = (Value.kReverse == orientationUpDoubleSolenoid.get());
    inputs.isRampClosed = (Value.kReverse == orientationRampDoubleSolenoid.get());
    inputs.isRunning = isRunning;
  }

  @Override
  public void toggleRampSolenoid() {
    orientationRampDoubleSolenoid.toggle();
    System.out.println("shit");
  }

  public void toggleUpSolenoid(){
    orientationUpDoubleSolenoid.toggle();
  }

  public void SetRampSolenoidState(boolean state){
    if(state){ orientationRampDoubleSolenoid.set(Value.kForward);}
    else{ orientationRampDoubleSolenoid.set(Value.kReverse);}
  }

  public void SetUpSolenoid(boolean state){
    if(state){ orientationUpDoubleSolenoid.set(Value.kForward);}
    else{ orientationUpDoubleSolenoid.set(Value.kReverse);}
  }

  public void disableMotors(){
    orientationRampMotor.set(ControlMode.Disabled, 0);
    orientationUpMotor.set(ControlMode.Disabled, 0);
  }
  
  @Override
  public void setPercent(double percent) {
    orientationRampMotor.set(ControlMode.PercentOutput, -percent + 0.1);
    orientationUpMotor.set(ControlMode.PercentOutput, percent);
  }

}
