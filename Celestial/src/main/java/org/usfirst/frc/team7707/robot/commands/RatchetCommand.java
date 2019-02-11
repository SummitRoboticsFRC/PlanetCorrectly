/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import org.usfirst.frc.team7707.robot.subsystems.RatchetSubsystem;
import org.usfirst.frc.team7707.robot.OI;

import edu.wpi.first.wpilibj.command.Command;

public class RatchetCommand extends Command {
  RatchetSubsystem ratchetSubsystem;

  public RatchetCommand(RatchetSubsystem ratchetSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.ratchetSubsystem = ratchetSubsystem;
    requires(ratchetSubsystem);
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    ratchetSubsystem.lift();
/*    ratchetSubsystem.getButtons();
    if (ratchetSubsystem.doBackDescend) {
      ratchetSubsystem.backDescend();
    }
    else if (ratchetSubsystem.doFrontDescend) {
      ratchetSubsystem.frontDescend();
    }
    else if (OI.leftTriggerPressed || OI.rightTriggerPressed){
      ratchetSubsystem.lift();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
