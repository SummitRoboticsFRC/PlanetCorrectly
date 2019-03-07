/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team7707.robot.subsystems.WidgetSubsystem;

public class DefaultWidgetCommand extends Command {
	WidgetSubsystem widgetSubsystem;

	public DefaultWidgetCommand(WidgetSubsystem widgetSubsystem) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.widgetSubsystem = widgetSubsystem;
		requires(widgetSubsystem);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		widgetSubsystem.setMoveStop(); // by default we stop the widget!
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
