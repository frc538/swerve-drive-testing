/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
  public static class CAN_ID {
    public static final int FRONT_LEFT_DRIVE = 2;
    public static final int FRONT_LEFT_TURN = 1;
    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int FRONT_RIGHT_TURN = 3;
    public static final int REAR_LEFT_DRIVE = 6;
    public static final int REAR_LEFT_TURN = 5;
    public static final int REAR_RIGHT_DRIVE = 7;
    public static final int REAR_RIGHT_TURN = 8;
  }

  public static final int FRONT_LEFT_ABS_ENCODER_ID = 1;
  public static final int FRONT_RIGHT_ABS_ENCODER_ID = 0;
  public static final int REAR_LEFT_ABS_ENCODER_ID = 2;
  public static final int REAR_RIGHT_ABS_ENCODER_ID = 3;
  
}
