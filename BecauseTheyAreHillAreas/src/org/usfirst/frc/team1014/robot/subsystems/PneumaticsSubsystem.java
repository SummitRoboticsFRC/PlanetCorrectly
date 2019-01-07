package org.usfirst.frc.team1014.robot.subsystems;

import org.usfirst.frc.team1014.robot.commands.PneumaticsStay;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PneumaticsSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	Solenoid cylinder = new Solenoid(0);
	
	
	Compressor c = new Compressor (0);
	boolean enabled = c.enabled();
	boolean pressureSwitch = c.getPressureSwitchValue();
	double current = c.getCompressorCurrent();
	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new PneumaticsStay());
    }
    
    public void solenoidValves(boolean status) {
    		cylinder.set(status);
    }
    
    public void compressorStatus(boolean status) {
    	c.setClosedLoopControl(status);
    	// c.setClosedLoopControl(false);
    }
}

