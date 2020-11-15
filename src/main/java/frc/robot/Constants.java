/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Swerve conversion factors
    public static final double VOLTAGE_TO_RAD_CONV_FACTOR = 2*Math.PI/3.3; 
    public static final double RAD_TO_DEG_CONV_FACTOR = 180/Math.PI;
    public static final double DEG_TO_RAD_CONV_FACTOR = Math.PI/180;
    public static final double RAD_TO_ENC_CONV_FACTOR = 0.0;//TODO:find the radian to enc factor

    //Swerve PID constants
    public static final double SWERVE_ROT_P_VALUE = 1.0;
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.0;
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0.0;
    public static final double SWERVE_ROT_FF_VALUE = 0.0;

}
