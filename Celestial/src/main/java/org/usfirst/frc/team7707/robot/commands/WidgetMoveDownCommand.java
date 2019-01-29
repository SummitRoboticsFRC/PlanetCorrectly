/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team7707.robot.subsystems.WidgetSubsystem;

public class WidgetMoveDownCommand extends Command {
	WidgetSubsystem widgetSubsystem;
	private double startTime;
	private double runTime;
	
    public WidgetMoveDownCommand(WidgetSubsystem widgetSubsystem, double runTime) {
    	this.widgetSubsystem = widgetSubsystem;
    	this.runTime = runTime;
    	requires(widgetSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startTime = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	widgetSubsystem.setMoveDown();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double now = Timer.getFPGATimestamp();
        if (now > (startTime + runTime)) {
        	return true;
        }
        return false;					// not finished yet
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
