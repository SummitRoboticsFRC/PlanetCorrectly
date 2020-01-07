/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team7707.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsCommand extends Command {
    PneumaticsSubsystem pneumaticsSubsystem;
  public PneumaticsCommand(PneumaticsSubsystem pneumaticsSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    requires(pneumaticsSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      pneumaticsSubsystem.compressorInit();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    pneumaticsSubsystem.updateCompressorStatus();
    Timer.delay(20.0);
    for (int i = 0; i < 4; i ++) {
      pneumaticsSubsystem.solenoidForward();
      Timer.delay(1.0);
      pneumaticsSubsystem.solenoidBack();
      Timer.delay(1.0);
    }
    //System.out.println("Push");
    //pneumaticsSubsystem.solenoidOff();
    //System.out.println("Pull");
    //pneumaticsSubsystem.solenoidOff();*/
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !pneumaticsSubsystem.getEnabled();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pneumaticsSubsystem.solenoidOff();
    pneumaticsSubsystem.compressorOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
