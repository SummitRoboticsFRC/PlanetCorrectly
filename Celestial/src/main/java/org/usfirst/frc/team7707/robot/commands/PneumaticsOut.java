package org.usfirst.frc.team1014.robot.commands;

import org.usfirst.frc.team1014.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PneumaticsOut extends Command {

	double time;
	
    public PneumaticsOut(double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.pneumaticsSubsystem);
    		this.time = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		setTimeout(time);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
   		Robot.pneumaticsSubsystem.solenoidValves(true);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
