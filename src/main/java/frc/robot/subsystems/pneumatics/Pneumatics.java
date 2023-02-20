package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private static Pneumatics instance;
  private PneumaticsIO io;
  private PneumaticsIOInputsAutoLogged inputs = new PneumaticsIOInputsAutoLogged();

  private Pneumatics(PneumaticsIO io) {
    this.io = io;
  }

  public static Pneumatics getInstance() {
    if (instance == null)
      instance = new Pneumatics(PneumaticsIORevPH.getInstance());
    return instance;
  }

  public void enableCompressor() {
    io.enableCompressor();
  }

  public void disableCompressor() {
    io.disableCompressor();
  }

  public double getPressure() {
    return io.getPressure();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
