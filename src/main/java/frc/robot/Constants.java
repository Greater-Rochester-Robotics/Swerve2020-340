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
    public static final double VOLTAGE_TO_RAD_CONV_FACTOR = 2*Math.PI/3.3; //TODO:Check that this is right(with Elec)
    public static final double RAD_TO_DEG_CONV_FACTOR = 180/Math.PI;
    public static final double DEG_TO_RAD_CONV_FACTOR = Math.PI/180;
    public static final double RAD_TO_ENC_CONV_FACTOR = 0.0;//TODO:find the radian to enc factor(Mech Team)
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;

    //Swerve dimension factors, distances from center of mass(units irrelivent, must all be the same though!)
    public static final double X_POSITIVE_DISTANCE_FROM_CENTER = 5;//TODO:find this distance(Mech Team)
    public static final double Y_POSITIVE_DISTANCE_FROM_CENTER = 5;
    public static final double X_NEGATIVE_DISTANCE_FROM_CENTER = -5;//THIS NUMBER MUST BE NEGATIVE!!!!!
    public static final double Y_NEGATIVE_DISTANCE_FROM_CENTER = -5;//THIS NUMBER MUST BE NEGATIVE!!!!!
    //These are computed from above values
    public static final double DISTANCE_TO_MODULE_0 = Math.sqrt(
        (X_POSITIVE_DISTANCE_FROM_CENTER*X_POSITIVE_DISTANCE_FROM_CENTER) +
        (Y_POSITIVE_DISTANCE_FROM_CENTER*Y_POSITIVE_DISTANCE_FROM_CENTER) );
    public static final double DISTANCE_TO_MODULE_1 = Math.sqrt(
        (X_NEGATIVE_DISTANCE_FROM_CENTER*X_NEGATIVE_DISTANCE_FROM_CENTER) +
        (Y_POSITIVE_DISTANCE_FROM_CENTER*Y_POSITIVE_DISTANCE_FROM_CENTER) );
    public static final double DISTANCE_TO_MODULE_2 = Math.sqrt(
        (X_NEGATIVE_DISTANCE_FROM_CENTER*X_NEGATIVE_DISTANCE_FROM_CENTER) +
        (Y_NEGATIVE_DISTANCE_FROM_CENTER*Y_NEGATIVE_DISTANCE_FROM_CENTER) );
    public static final double DISTANCE_TO_MODULE_3 = Math.sqrt(
        (X_POSITIVE_DISTANCE_FROM_CENTER*X_POSITIVE_DISTANCE_FROM_CENTER) +
        (Y_NEGATIVE_DISTANCE_FROM_CENTER*Y_NEGATIVE_DISTANCE_FROM_CENTER) );
    //this is an array of unit vectors(a vector of length one)
    public static final double[][] MODULE_UNIT_VECTORS = new double[][]{
            {X_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_0 , Y_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_0},
            {X_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_1 , Y_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_1},
            {X_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_2 , Y_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_2},
            {X_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_3 , Y_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_3}
        };

    //Swerve PID constants
    public static final double SWERVE_ROT_P_VALUE = 1.0;
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.0;
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0.0;
    public static final double SWERVE_ROT_FF_VALUE = 0.0;

    //SparkMAX motor controllers
    public static final int FRONT_LEFT_MOVE_MOTOR = 1;//module 0
    public static final int FRONT_LEFT_ROTATE_MOTOR = 2;//module 0
    public static final int FRONT_LEFT_ROTATE_SENSOR = 10;//module 0
    public static final int REAR_LEFT_MOVE_MOTOR = 3;//module 1
    public static final int REAR_LEFT_ROTATE_MOTOR = 4;//module 1
    public static final int REAR_LEFT_ROTATE_SENSOR = 11;//modle 1
    public static final int REAR_RIGHT_MOVE_MOTOR = 5;//module 2
    public static final int REAR_RIGHT_ROTATE_MOTOR = 6;//module 2
    public static final int REAR_RIGHT_ROTATE_SENSOR = 12;//module 2
    public static final int FRONT_RIGHT_MOVE_MOTOR = 7;//module 3
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 8;//module 3
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 9;//module 3 
}
