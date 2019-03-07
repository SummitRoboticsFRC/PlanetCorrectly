/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team7707.robot.subsystems.PneumaticsSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PneumaticsCommand extends Command {

  PneumaticsSubsystem pneumaticsSubsystem;

  public PneumaticsCommand(PneumaticsSubsystem pneumaticsSubsystem) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        requires(pneumaticsSubsystem);
        setInterruptible(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		pneumaticsSubsystem.compressorStatus();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		pneumaticsSubsystem.solenoidValves();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
