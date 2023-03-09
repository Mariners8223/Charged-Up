package frc.robot.subsystems.orientation;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.OrientationConstants;

public class OrientationIOVictorSPX implements OrientationIO {
  private static OrientationIOVictorSPX instance;
  private VictorSPX elevatedMotor;
  private VictorSPX rampMotor;
  private DoubleSolenoid rampSolenoid;
  private DoubleSolenoid elevatedSolenoid;
  public boolean isRunning; // I'm too lazy to implement an efficient system someone else do it

  private OrientationIOVictorSPX() {
    elevatedMotor = new VictorSPX(OrientationConstants.ORIENTATION_ELEVATED_MOTOR);
    rampMotor = new VictorSPX(OrientationConstants.ORIENTATION_RAMP_MOTOR);
    rampSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
        OrientationConstants.ORIENTATION_RAMP_SOLENOID_PORTS[0], OrientationConstants.ORIENTATION_RAMP_SOLENOID_PORTS[1]);
    elevatedSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
        OrientationConstants.ORIENTATION_ELEVATED_SOLENOID_PORTS[0], OrientationConstants.ORIENTATION_ELEVATED_SOLENOID_PORTS[1]);
    isRunning = false;
    rampSolenoid.set(Value.kReverse);
    elevatedSolenoid.set(Value.kReverse);
  }

  public static OrientationIOVictorSPX getInstance() {
    if (instance == null)
      instance = new OrientationIOVictorSPX();
    return instance;
  }

  @Override
  public void updateInputs(OrientationIOInputs inputs) {
    inputs.isOrientaionClosed = (Value.kReverse == elevatedSolenoid.get());
    inputs.isRampClosed = (Value.kReverse == rampSolenoid.get());
    inputs.isRunning = isRunning;
  }

  @Override
  public void toggleRampSolenoid() {
    rampSolenoid.toggle();
  }

  public void toggleElevatedSolenoid(){
    elevatedSolenoid.toggle();
  }


  public void raiseOrientation() { elevatedSolenoid.set(Value.kReverse); }
  public void lowerOrientation() { elevatedSolenoid.set(Value.kForward); }
  public void raiseRamp() { rampSolenoid.set(Value.kReverse); }
  public void lowerRamp() { rampSolenoid.set(Value.kForward); }


  public void disableMotors(){
    rampMotor.set(ControlMode.Disabled, 0);
    elevatedMotor.set(ControlMode.Disabled, 0);
  }
  
  @Override
  public void setPercent(double percent) {
    rampMotor.set(ControlMode.PercentOutput, -percent + 0.1);
    elevatedMotor.set(ControlMode.PercentOutput, percent);
  }

}
