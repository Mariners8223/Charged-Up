package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticsIORevPH implements PneumaticsIO {
  private static PneumaticsIORevPH instance;
  private Compressor compressor;
  
  private PneumaticsIORevPH() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);
  }

  public static PneumaticsIORevPH getInstance() {
    if (instance == null)
      instance = new PneumaticsIORevPH();
    return instance;
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    inputs.currentPSI = compressor.getPressure();
    inputs.isEnabled = compressor.isEnabled();
  }

  @Override
  public double getPressure() {
    return compressor.getPressure();
  }

  @Override
  public void enableCompressor() {
    compressor.enableDigital();
  }

  @Override
  public void disableCompressor() {
    compressor.disable();
  } 

}
