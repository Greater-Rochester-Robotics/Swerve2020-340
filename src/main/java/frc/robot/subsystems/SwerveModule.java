/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANAnalog;

import frc.robot.Constants;



/**
 * This is the class containing both motor controllers and all functions needed to run one swerve module.
 * This is a subsystem for testing purposes, and maybe to use the periodic function to reduce calls to the rotation sensor.
 */
public class SwerveModule extends SubsystemBase {
    private CANSparkMax driveMotor;
    private CANSparkMax rotationMotor;
    private CANEncoder rotationEncoder;
    private CANAnalog rotationSensor;
    private CANPIDController rotatePID;
    private boolean isInverted = false;//this is for a future function

    /**
     * Creates a new SwerveModule object
     * 
     * @param driveMotorID The CAN ID of the SparkMax connected to the drive motor(expecting NEO)
     * @param rotateMotorID The CAN ID of the SparkMax connected to the module rotation motor(expecting NEO 550)
     */  
    public SwerveModule(int driveMotorID,int rotationMotorID){
        //TODO:change this to a TalonFX, check all uses of driveMotor for the right syntax
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();//reset the motor controller, wipe old stuff

        rotationMotor = new CANSparkMax(rotationMotorID , MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();//reset the motor controller, wipe old stuff

        //TODO:change this Analog to a CANCoder(comment out), this requires an additional constructor parameter 
        rotationSensor = rotationMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
        rotationSensor.setPositionConversionFactor(Constants.VOLTAGE_TO_RAD_CONV_FACTOR);

        rotationEncoder = rotationMotor.getEncoder();
        rotatePID = rotationMotor.getPIDController();
        rotatePID.setFeedbackDevice(rotationEncoder);

        //set the PID values for the Encoder controlled rotation
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
        driveMotor.set(value);//*(isInvertted?-1:0));
    }

    //TODO:create access to the driveMotor encoder count

    //TODO:create a reset for the driveMotor encoder

    /**
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getPosInDeg(){ 
        return getPosInRad()*Constants.RAD_TO_DEG_CONV_FACTOR;
    }

    /**
     * If this is too resource intensive, switch to a periodic call,
     *  and replace with a poll of said variable
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad(){
        //TODO:fix math to allign with CANCoder outputs
        return rotationSensor.getPosition() - Math.PI;//(isInverted?0:Math.PI));
        //TODO:Above has to be checked, if the sensor is positive clockwise, fix(Need Robot)
        // double currentAngle = rotationSensor.getPosition();
        // if(isInverted){
        //     if(currentAngle <= Math.PI){
        //         return currentAngle;
        //     }else{
        //         return Constants.TWO_PI-currentAngle;
        //     }
        // }else{
        //     return rotationSensor.getPosition() - Math.PI;
        // }
    }

    /**
     * this is a function meant for testing by getting the count from
     *  the rotational encoder which is internal to the NEO550.
     * @return the encoder count(no units, naturally just the count)
     */
    public double getEncCount(){
        return rotationEncoder.getPosition();
    }

    /**
     * set the setpoint for the module rotation
     * @param targetPos a value between -PI and PI, PI is counter-clockwise, 0.0 is forward 
     */
    public void setPosInRad(double targetPos){
        double posDiff = targetPos - getPosInRad();
        double absDiff = Math.abs(posDiff);
        if(absDiff > Math.PI){
            //the distance the other way around the circle
            posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        }

        // //This is for inverting the motor if target angle is 90-270 degrees away (not ready yet)
        // //To fix going the wrong way around the circle
        // if(absDiff >= Constants.THREE_PI_OVER_TWO){
        //     //the distance the other way around the circle
        //     posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        // //if between 90 and 270 invert the motor
        // }else if(absDiff < Constants.THREE_PI_OVER_TWO && absDiff > Constants.PI_OVER_TWO){
        //     //switch the motor inversion
        //     isInverted = !isInverted;
        //     //Since inverted, recompute everything
        //     posDiff = targetPos - getPosInRad();
        //     absDiff = Math.abs(posDiff);
        //     if(absDiff > Constants.THREE_PI_OVER_TWO){
        //         //the distance the other way around the circle
        //         posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        //     }
        // }
     
        //Convert the shortest distance to encoder value(use convertion factor) 
        double targetEncDistance = posDiff*Constants.RAD_TO_ENC_CONV_FACTOR;
        //add the encoder distance to the current encoder count
        double outputEncValue = targetEncDistance + rotationEncoder.getPosition();
        
        //Set the setpoint using setReference on the PIDController
        rotatePID.setReference(outputEncValue, ControlType.kPosition);
    }

    /**
     * this method is used to stop the module completely.
     */
    public void stopAll(){
        driveMotor.set(0.0);
        rotatePID.setReference(0.0,ControlType.kVoltage);
    }
}
