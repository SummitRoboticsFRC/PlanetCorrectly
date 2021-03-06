/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import org.usfirst.frc.team7707.robot.RobotMap;
<<<<<<< HEAD
import org.usfirst.frc.team7707.robot.RobotMap.LiftStatus;
import org.usfirst.frc.team7707.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class DefaultLiftCommand extends Command {

  LiftSubsystem liftSubsystem;
  DoubleSupplier speed;
  //BooleanSupplier buttonL1, buttonL2, buttonL3;

  public DefaultLiftCommand(LiftSubsystem liftSubsystem, DoubleSupplier speed) { // paramater for buttons removed
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.liftSubsystem = liftSubsystem;
    this.speed = speed;
    /* this.buttonL1 = buttonL1;
    this.buttonL2 = buttonL2;
    this.buttonL3 = buttonL3; */
=======
import org.usfirst.frc.team7707.robot.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class DefaultLiftCommand extends Command {
  LiftSubsystem liftSubsystem;

  public DefaultLiftCommand(LiftSubsystem liftSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.liftSubsystem = liftSubsystem;
>>>>>>> 9f4c061d26bce129ec3bc59cb0549afc523ddbf8
    requires(liftSubsystem);
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
<<<<<<< HEAD
    
    liftSubsystem.updateValues();

    if(speed.getAsDouble() > 0.1 || speed.getAsDouble() < -0.1) {
      liftSubsystem.lift(LiftStatus.LIFT_MANUAL);
    } 

    //john

    /* else if (buttonL1.getAsBoolean()) {
      liftSubsystem.lift(LiftStatus.LIFT_LEVEL_1);
    }

    else if (buttonL2.getAsBoolean()) {
      liftSubsystem.lift(LiftStatus.LIFT_LEVEL_2);
    }

    else if (buttonL3.getAsBoolean()) {
      liftSubsystem.lift(LiftStatus.LIFT_LEVEL_3);
    } else {
      liftSubsystem.lift(LiftStatus.NO_COMMAND);
    } */

=======
    liftSubsystem.lift(RobotMap.LiftStatus.LIFT_MANUAL);
>>>>>>> 9f4c061d26bce129ec3bc59cb0549afc523ddbf8
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
