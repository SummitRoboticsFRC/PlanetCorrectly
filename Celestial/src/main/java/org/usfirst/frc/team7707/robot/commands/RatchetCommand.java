/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import org.usfirst.frc.team7707.robot.subsystems.RatchetSubsystem;
import org.usfirst.frc.team7707.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;

public class RatchetCommand extends Command {

  RatchetSubsystem ratchetSubsystem;
  Joystick driverInput;

  public RatchetCommand(RatchetSubsystem ratchetSubsystem, Joystick driverInput) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.ratchetSubsystem = ratchetSubsystem;
    this.driverInput = driverInput;
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

    boolean leftButton = driverInput.getRawButton(RobotMap.buttonL);
    double leftTrigger = driverInput.getRawAxis(RobotMap.leftTrigger);
    if (leftButton) {
      ratchetSubsystem.setFrontMotorSpeed(1);
    } else if (leftTrigger > 0.7) {
      ratchetSubsystem.setFrontMotorSpeed(-1);
    } else {
      ratchetSubsystem.setFrontMotorSpeed(0.0);
    }

    boolean rightButton = driverInput.getRawButton(RobotMap.buttonR);
    double rightTrigger = driverInput.getRawAxis(RobotMap.rightTrigger);
    if (rightButton) {
      ratchetSubsystem.setBackMotorSpeed(-1.0);
    } else if (rightTrigger > 0.7) {
      ratchetSubsystem.setBackMotorSpeed(1.0);
    } else {
      ratchetSubsystem.setBackMotorSpeed(0.0);
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
    ratchetSubsystem.setBackMotorSpeed(0.0);
    ratchetSubsystem.setFrontMotorSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
