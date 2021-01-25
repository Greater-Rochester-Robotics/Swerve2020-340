/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.analog.adis16448.frc.ADIS16448_IMU.IMUAxis;
/**
 * This is the subsystem that governs the four swerve module objects.
 *  In this class, the positive x-axis is toward the front of the robot,
 *  and the positive y-axis is toward the left side. All angles are
 *  measured from the positive x-axis and positive angles are couter
 *  clockwise from that axis. The modules are numbered, starting at 
 *  module0 for the left front module continuing counter clockwise.
 *  This makes the rear left module1, the rear right module2, and the
 *  front right module3.
 */
public class SwerveDrive extends SubsystemBase {
  private static SwerveModule swerveModules[];
  private static SwerveModule frontLeft,rearLeft,rearRight,frontRight;
  public ADIS16448_IMU imu;
  
  /**
   * This enumeration clarifies the numbering of the swerve module for new users.
   * frontLeft  | 0
   * rearLeft   | 1
   * rearRight  | 2
   * frontRight | 3
   */
  public enum kSwerveModule{
    frontLeft(0) , rearLeft(1) , rearRight(2) , frontRight(3);
    private int num;
    private kSwerveModule(int number){
      num = number;
    }
    public int getNumber() {
			return num;
		}
  }
  
  /**
   * Creates a new SwerveDrive.
   */
  public SwerveDrive() {
    //TODO:properly construct the swerve modules, and put them in an array, int already in Constants
    // frontLeft = new SwerveModule();
    // rearLeft = new SwerveModule();
    // rearRight = new SwerveModule();
    // frontRight = new SwerveModule();
    ////This may seem repetitive, but it makes clear which module is which.
    // swerveModules = new SwerveModule[]{
    //   frontLeft,
    //   rearLeft,
    //   rearRight,
    //   frontRight
    // };
    
    //TODO:Construct the IMU object, alreeady named imu

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed){
    double[] targetMoveVector = { forwardSpeed , strafeSpeed };//the direction we want the robot to move

    //create a 2d array for the goal output of each module(in vector component form)
    double[][] targetModuleVectors = new double[2][4];
    //create a vector for each module, one at a time
    for(int i=0 ; i<4 ; i++){
      //compute the x-component of the vector by adding the targetVector to the cross product with rotspeed
      targetModuleVectors[0][i] =
        targetMoveVector[0] - (rotSpeed*Constants.MODULE_UNIT_VECTORS[1][i] );
      
      //compute the y-component of the vector by adding the targetVector to the cross product with rotspeed
      targetModuleVectors[1][i] = 
        targetMoveVector[1] + (rotSpeed*Constants.MODULE_UNIT_VECTORS[0][i] );//TODO:check if this sign is right
    }

    //generates angles for each module
    double[] targetModuleAngles = new double[4];
    for(int i=0 ; i<4 ; i++){
      targetModuleAngles[i] = Math.atan2( targetModuleVectors[1][i] , targetModuleVectors[0][i] );
    }

    //TODO:assign the angles to the swerve drive modules (use a for loop and setPosInRad())

    double[] curAngles;//TODO:pull the current angles of the modules(use a for loop and getPosInRad())
    

    double[] targetMotorSpeeds = new double[4];
    //create a variable so we can find the maxSpeed
    double maxSpeed = 0.0;
    
    for(int i=0 ; i<4 ; i++){
      //find the length(power) for each direction vector
      targetMotorSpeeds[i] = Math.sqrt(
        (targetModuleVectors[0][i]*targetModuleVectors[0][i]) +
        (targetModuleVectors[1][i]*targetModuleVectors[1][i])   );
      //find and store the largest speed(all speeds are positive)
      if(maxSpeed < targetMotorSpeeds[i]){
        maxSpeed = targetMotorSpeeds[i];
      }
    }

    //normalize all speeds, by dividing by the largest, if largest is greater than 1
    if(maxSpeed > 1){
      for(int i=0 ; i<4 ; i++){
        targetMotorSpeeds[i] = targetMotorSpeeds[i]/maxSpeed;
      }
    }

    //Reduce power to motors until they align with the target angle(might remove later)
    for(int i=0 ; i<4 ; i++){
      targetMotorSpeeds[i] = targetMotorSpeeds[i]*Math.cos(targetModuleAngles[i]-curAngles[i]);
    }

    
    //TODO:assign output to each module(use a for loop with targetMotorSpeeds[])

  }

  /**
   * 
   * @param awaySpeed
   * @param lateralSpeed
   * @param rotSpeed
   */
  public void driveFieldCentric(double awaySpeed, double lateralSpeed, double rotSpeed){
    double robotForwardSpeed = 0.0;//TODO:use Math.cos(this.getGyroInRad())*awaySpeed plus Math.sin() * lateralSpeed
    double robotStrafeSpeed = 0.0;//TODO:add the reverse of the above, cos with latera Speed and sin with awaySpeed
    this.driveRobotCentric( robotForwardSpeed , robotStrafeSpeed , rotSpeed );
  }

  /**
   * This function is meant to drive one module at a time for testing purposes.
   * @param moduleNumber
   * @param moveSpeed
   * @param rotatePos
   */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos){
    //TODO:test that moduleNumber is between 0-3, return if not(return;)
    //TODO:write code to drive one module in a testing form
  }

  /**
   * A function that allows the user to reset the gyro
   */
  public void resetGyro(){
    //TODO:Reset the gyro(zero it)
  }

  /**
   * this polls the onboard gyro, which, when the robot boots, assumes and angle of zero
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
    return 0.0;//TODO:Pull and return gyro in radians
  }

  /**
   * this polls the onboard gyro, which, when the robot boots, assumes and angle of zero
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return 0.0;//TODO:Pull gyro in radians and convert to degrees
  }

  //TODO:create Stop all method to stop all the motors using the stopAll method

}
