/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANAnalog;

/**
 * This is the class containing both motor controllers and all functions needed to run one swerve module.
 */
public class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax rotationMotor;
    private CANEncoder rotationEncoder;
    private CANAnalog rotationSensor;
    private CANPIDController rotatePID;
    // private boolean isInverted = false;//this is for a future function

    /**
     * Creates a new SwerveModule object
     * 
     * @param driveMotorID The CAN ID of the SparkMax connected to the drive motor(expecting NEO)
     * @param rotateMotorID The CAN ID of the SparkMax connected to the module rotation motor(expecting NEO 550)
     */  
    public SwerveModule(int driveMotorID,int rotationMotorID){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();//reset the motor controller, wipe old stuff

        rotationMotor = new CANSparkMax(rotationMotorID , MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();//reset the motor controller, wipe old stuff
        rotationEncoder = rotationMotor.getEncoder();
        rotationSensor = rotationMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
        rotationSensor.setPositionConversionFactor(Constants.VOLTAGE_TO_RAD_CONV_FACTOR);

        rotatePID = rotationMotor.getPIDController();
        rotatePID.setFeedbackDevice(rotationEncoder);

        rotatePID.setP(Constants.SWERVE_ROT_P_VALUE);
        rotatePID.setI(Constants.SWERVE_ROT_I_VALUE);
        rotatePID.setD(Constants.SWERVE_ROT_D_VALUE);
        rotatePID.setIZone(Constants.SWERVE_ROT_I_ZONE_VALUE);
        rotatePID.setFF(Constants.SWERVE_ROT_FF_VALUE);
    }

    /**
     * Set the speed of the drive motor
     * 
     * @param value a number between -1.0 and 1.0, where 0.0 is not moving
     */
    public void setDriveMotor(double value){
        driveMotor.set(value);
    }

    /**
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getPosInDeg(){ 
        return getPosInRad()*Constants.RAD_TO_DEG_CONV_FACTOR;
    }

    /**
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad(){
        return rotationSensor.getPosition() - Math.PI;//this has to be checked, if the sensor is positive clockwise, fix
    }

    /**
     * set the setpoint for the module rotation
     * @param targetPos a value between -PI and PI, PI is counter-clockwise, 0.0 is forward 
     */
    public void setPosInRad(double targetPos){
        double posDiff = targetPos - getPosInRad();
        //TODO: Check you have the shortest distance(circle, eg the shortest distance between -2 and 2 is not going through 0
        //if abs posDiff > Math.PI (incomplete-very complicated, still unsure)
        //TODO: Convert the shortest distance to encoder value(use convertion factor) 
        //TODO: add the encoder distance to the current encoder count

        //TODO: set the setpoint using setReference on the PIDController
        // rotatePID.setReference(targetPos, ControlType.kPosition);
    }    
}
