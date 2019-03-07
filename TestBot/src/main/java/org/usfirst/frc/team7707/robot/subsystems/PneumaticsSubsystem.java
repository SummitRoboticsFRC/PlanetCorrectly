/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class PneumaticsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DoubleSolenoid cylinder = new DoubleSolenoid(1, 2);
  DoubleSolenoid cylinder1 = new DoubleSolenoid(1, 2);
	
	
	Compressor c = new Compressor (0);
	boolean enabled = c.enabled();
	boolean pressureSwitch = c.getPressureSwitchValue();
  double current = c.getCompressorCurrent();
  
  Compressor c1 = new Compressor (1);
	boolean enabled1 = c1.enabled();
	boolean pressureSwitch1 = c1.getPressureSwitchValue();
	double current1 = c1.getCompressorCurrent();
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void solenoidValves() {
        cylinder.set(DoubleSolenoid.Value.kForward);
        cylinder1.set(DoubleSolenoid.Value.kForward);
    }
    
    public void compressorStatus() {
    	c.setClosedLoopControl(true);
    	//c.setClosedLoopControl(false);
    }

}
