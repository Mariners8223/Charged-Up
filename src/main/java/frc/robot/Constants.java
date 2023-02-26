package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;
  public static final double fullThres = 0.99;
  public static final double heldThres = 0.1;
  public static final boolean IsLimeLightAprilTags = false;
  static class FieldConstants {
    static final double length = Units.feetToMeters(54);
    static final double width = Units.feetToMeters(27);
  }
  public static final int MOTORS_CHECKED_PER_TICK = 1;
  public static final int SRX_MAG_COUNTS_PER_REVOLUTION = 1024;
  public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  public static final Transform3d robotToLimeLight = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static class RobotConstants {
    public static final int ARM_ROTATION_MOTOR = 1;
    public static final int ARM_EXTENSION_MOTOR = 2;
    public static final int GRIPPER_MOTOR = 3;
    public static final int ORIENTATION_LEFT_MOTOR = 4;
    public static final int ORIENTATION_RIGHT_MOTOR = 5;
    public static final int[] ORIENTATION_DOUBLE_SOLENOID_PORTS = {0, 1};
    public static final int[] GRIPPER_DOUBLE_SOLENOID_PORTS = {8, 9};

  }

  public static class TempConstants{
    public static final int ENGINE1_ID = 1;
    public static final int ENGINE2_ID = 2;
    public static final int ENGINE3_ID = 3;
    public static final int ENGINE4_ID = 4;

  }

  public static class GripperConstants {
    public static final double GRIPPER_KP = 0.1;
    public static final double GRIPPER_SPEED = 1.0;
    public static final double GRIPPER_GEAR_RATIO = 0.625;
    public static final double GRIPPER_TOLERANCE = Units.degreesToRadians(2);
  }

  public static class OrientationConstants {
  }

  public static class ArmConstants {
    public static final double ARM_KA = 0.0;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KG = 0.0;
    public static final double ARM_KS = 0.0;

    public static final double ARM_ROTATION_KP = 0.8;
    public static final double ARM_ROTATION_KI = 0.5;
    public static final double ARM_ROTATION_KD = 0.3;
    public static final double ARM_ROTATION_KF = 0.5;
    public static final double ARM_ROTATION_TOLERANCE = Units.degreesToRotations(3);
    public static final double ARM_EXTENSION_KP = 0.8;
    public static final double ARM_EXTENSION_KI = 0.5;
    public static final double ARM_EXTENSION_KD = 0.3;

    public static final double ARM_ROTATION_GEAR_RATIO = 776.25;
    public static final double ARM_EXTENSION_GEAR_RATIO = 2.3;
    public static final double PINION_CIRCUMFERENCE_INCHES = 2.358268;
    public static final double PINION_CIRCUMFERENCE_METERS = Units.inchesToMeters(PINION_CIRCUMFERENCE_INCHES);
    public static final double PINION_RADIUS_INCHES = PINION_CIRCUMFERENCE_INCHES / 2;
    public static final double PINION_RADIUS_METERS = PINION_RADIUS_INCHES / 2 ;
    public static final double DISTANCE_PER_REVOLUTION = PINION_CIRCUMFERENCE_INCHES * Math.PI;

    public static final double ARM_EXTENSION_TOLERENCE = Units.metersToInches(0.05) * Constants.SRX_MAG_COUNTS_PER_REVOLUTION * ArmConstants.ARM_EXTENSION_GEAR_RATIO / ArmConstants.DISTANCE_PER_REVOLUTION;

    
  }
}