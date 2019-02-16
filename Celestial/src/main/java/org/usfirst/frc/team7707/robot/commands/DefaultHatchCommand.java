/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;
import org.usfirst.frc.team7707.robot.subsystems.HatchSubsystem;
import org.usfirst.frc.team7707.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;

public class DefaultHatchCommand extends Command {
HatchSubsystem hatchSubsystem;
Joystick driverInput;

  public DefaultHatchCommand(HatchSubsystem hatchSubsystem, Joystick driverInput) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.hatchSubsystem = hatchSubsystem;
    this.driverInput = driverInput;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (driverInput.getRawButton(RobotMap.buttonY)) {
      hatchSubsystem.setHatchSpeed(1.0);
    } else if (driverInput.getRawButton(RobotMap.buttonA)) {
      hatchSubsystem.setHatchSpeed(-1.0);
    } else {
      hatchSubsystem.setHatchSpeed(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    hatchSubsystem.setHatchSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
