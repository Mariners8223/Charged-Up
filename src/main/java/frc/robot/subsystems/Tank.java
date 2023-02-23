package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TemporaryConstants;

public class Tank extends SubsystemBase {
  //! THIS IS TEMPORARY AND SHOULD BE DELETED AFTER THE 24TH OF FEB.

  private static Tank instance;
  private VictorSPX leftLeader;
  private VictorSPX leftFollower;
  private VictorSPX rightLeader;
  private VictorSPX rightFollower;

  private AHRS NavX;

  private Tank() {
    leftLeader = new VictorSPX(TemporaryConstants.LEFT_LEADER);
    leftFollower = new VictorSPX(TemporaryConstants.LEFT_FOLLOWER);
    rightLeader = new VictorSPX(TemporaryConstants.RIGHT_LEADER);
    rightFollower = new VictorSPX(TemporaryConstants.RIGHT_FOLLOWER);
    
    NavX = new AHRS(SPI.Port.kMXP);
    NavX.calibrate();

    rightLeader.setInverted(InvertType.InvertMotorOutput);
    rightFollower.setInverted(InvertType.FollowMaster);
    leftLeader.setInverted(InvertType.None);
    leftFollower.setInverted(InvertType.FollowMaster);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
  }

  public static Tank getInstance() {
    if (instance == null)
      instance = new Tank();
    return instance;
  }

  public void Drive(double xSpeed, double rot) {
    leftLeader.set(ControlMode.PercentOutput, xSpeed+rot);
    rightLeader.set(ControlMode.PercentOutput, xSpeed+rot);
  }

  public void resetGyro() { NavX.reset(); }



  @Override
  public void periodic() {
    SmartDashboard.putData(NavX);
  }
}
