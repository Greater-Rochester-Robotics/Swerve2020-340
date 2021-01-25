/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command to test one module at a time, it takes that module as a constructor arguement
 */
public class DriveOneModule extends CommandBase {
  private int moduleNum;
  /**
   * Creates a new DriveOneModule.
   */
  public DriveOneModule(int moduleNumber) {
    //TODO:addRequirements use addRequirements() and pull the subSystem object from RobotContainer
    //TODO:assign the passed module number to the field of similiar name
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:Write this code, drive one module test. Using driveOneModule
    //pull the degree from the Dpad and pass to module
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RODO:Make sure to stop the module, by using a stopAll method from SwerveDrive
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
