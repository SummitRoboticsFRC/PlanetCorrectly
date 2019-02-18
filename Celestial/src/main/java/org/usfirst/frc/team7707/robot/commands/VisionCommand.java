/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import org.usfirst.frc.team7707.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class VisionCommand extends Command {
  private VisionSubsystem visionSubsystem;
  public VisionCommand(VisionSubsystem visionSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.visionSubsystem=visionSubsystem;
    requires(visionSubsystem);
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    visionSubsystem.makePath();
    visionSubsystem.PostToDashBoard();
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
