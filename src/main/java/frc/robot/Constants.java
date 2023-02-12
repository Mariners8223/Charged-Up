// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final int MOTORS_CHECKED_PER_TICK = 1;
  public static final int SRX_MAG_COUNTS_PER_REVOLUTION = 1024;

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
  }

  public static class GripperConstants {
    public static final double GRIPPER_KP = 0.1;
    public static final double GRIPPER_SPEED = 1.0;
    public static final double GRIPPER_GEAR_RATIO = 0.0;
  }

  public static class ArmConstants {
    public static final double ARM_KA = 0.0;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KG = 0.0;
    public static final double ARM_KS = 0.0;
  }
}
