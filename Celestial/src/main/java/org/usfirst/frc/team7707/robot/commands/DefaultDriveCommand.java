/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team7707.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.VisionSubsystem;
import org.usfirst.frc.team7707.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;
public class DefaultDriveCommand extends Command {
  DriveSubsystem driveSubsystem;
  VisionSubsystem vision; 
  //Joystick activateVision; 
  public DefaultDriveCommand(DriveSubsystem driveSubsystem, VisionSubsystem vison) {
    // Use requires() here to declare subsystem dependencies
    this.driveSubsystem = driveSubsystem;
    this.vision = vision;
    //this.activateVision = activateVision; 
    requires(driveSubsystem);
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveSubsystem.drive();
    /* if(activateVision.getRawButton(RobotMap.buttonLThumb)){
      driveSubsystem.AlignRobotDrive(vision.firstTurn, vision.driveDist);
    } */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveSubsystem.driveStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
