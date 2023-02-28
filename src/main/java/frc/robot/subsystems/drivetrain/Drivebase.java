package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.gyro.GyroIONavX2;
import frc.robot.subsystems.drivetrain.module.ModuleIOFXAndSMax;
import frc.robot.subsystems.drivetrain.module.ModuleIOInputsAutoLogged;

public class Drivebase extends SubsystemBase {

  // TODO: COMMENT

  private static Drivebase instance;

  private GyroIONavX2 gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private ModuleIOFXAndSMax[] swerveModules = new ModuleIOFXAndSMax[1];
  private ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] { new ModuleIOInputsAutoLogged(),
      new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
      new ModuleIOInputsAutoLogged() };

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private boolean brakeMode = false;

  private Drivebase(GyroIONavX2 gyroIO, ModuleIOFXAndSMax[] Modules) {
    swerveModules[0] = Modules[0];
    // swerveModules[1] = Modules[1];
    // swerveModules[2] = Modules[2];
    // swerveModules[3] = Modules[3];
    this.gyroIO = gyroIO;

    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    // odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyroIO.getAngleDeg()), new SwerveModulePosition[] {
    //   swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
    //   swerveModules[2].getModulePosition(), swerveModules[3].getModulePosition()
    // });
  }

  public static Drivebase getInstance() {
    if (instance == null) {
      instance = new Drivebase(GyroIONavX2.getInstance(),
          new ModuleIOFXAndSMax[] { new ModuleIOFXAndSMax(Drivetrain.TLModule),
              // new ModuleIOFXAndSMax(Drivetrain.TRModule),
              // new ModuleIOFXAndSMax(Drivetrain.BLModule), new ModuleIOFXAndSMax(Drivetrain.BRModule) });
          });
    }
    ;
    return instance;
  }

  public void Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
        fieldRelative && gyroIO.isConnected()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyroIO.getAngleDeg()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
        Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (int i = 0; i < 1/*swerveModules.length*/; i++) {
      swerveModules[0].setDesiredState(moduleStates[0]);
    }
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    for (int i = 0; i< 1 /* swerveModules.length */; i++) {
      swerveModules[i].setDesiredState(moduleStates[i]);
    }
  }

  public void setModulesState(SwerveModuleState[] states) {
    for (int i = 0; i < 1; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        Constants.Drivetrain.TRModule.position,
        Constants.Drivetrain.TLModule.position,
        Constants.Drivetrain.BRModule.position,
        Constants.Drivetrain.BLModule.position
    };
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (int i = 0; i < 1; i++) {
      swerveModules[i].updateInputs(moduleInputs[i]);
      swerveModules[i].setDriveBrakeMode(brakeMode);
      Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
          moduleInputs[i]);
    }

    // odometry.update(getRotation2d(), new SwerveModulePosition[] {
    //     swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
    //     swerveModules[2].getModulePosition(), swerveModules[3].getModulePosition()
    // });

    
    Logger.getInstance().recordOutput("Odometry", getPose());

    SmartDashboard.putNumber("cancoder angle", swerveModules[0].getAbsSteeringPos());

  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyroIO.getAngleDeg());
  }

  public void resetOdometry(Pose2d initalPose2d) {
    resetOdometry(getRotation2d(),
        new SwerveModulePosition[] { swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
            swerveModules[2].getModulePosition(), swerveModules[3].getModulePosition() },
        initalPose2d);
  }

  public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    odometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  public Pose2d getPose() {
    // return odometry.getPoseMeters();
    return new Pose2d();
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  public boolean getBrake() {
    return this.brakeMode;
  }

  public void setBrkae(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }
}
