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

import org.usfirst.frc.team7707.robot.commands.PneumaticsCommand;

/**
 * Add your docs here.
 */
public class PneumaticsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DoubleSolenoid cylinder; //= new DoubleSolenoid(1, 2);
  //DoubleSolenoid cylinder1 = new DoubleSolenoid(1, 2);
    //Compressor compressor;
	
	Compressor compressor; //= new Compressor (0);
	boolean enabled, pressureSwitch, closedLoop; //= compressor.getPressureSwitchValue();
    double current; //= compressor.getCompressorCurrent(); /*= compressor.enabled()*/ 
  
    public PneumaticsSubsystem(DoubleSolenoid cylinder, Compressor compressor) {
        this.cylinder = cylinder;
        this.compressor = compressor;
        this.enabled = true;
    }

    public void compressorInit() {
        
        compressor.start();
        compressor.setClosedLoopControl(true); 
        //c.setClosedLoopControl(false);
    }

    public void compressorOff() {
        compressor.setClosedLoopControl(false);
        compressor.close();
    }

    public void updateCompressorStatus() {
        //enabled = compressor.enabled();
        pressureSwitch = compressor.getPressureSwitchValue();
        //closedLoop = compressor.getClosedLoopControl();
        current = compressor.getCompressorCurrent();
    }

    public boolean getPressureSwitchValue() {
        //return compressor.getPressureSwitchValue();
        return true;
    }

    public boolean getEnabled() {
        return enabled;
    }

  /*Compressor c1 = new Compressor (1);
	boolean enabled1 = c1.enabled();
	boolean pressureSwitch1 = c1.getPressureSwitchValue();
	double current1 = c1.getCompressorCurrent();*/

    
    public void solenoidForward() {
        cylinder.set(DoubleSolenoid.Value.kForward);
        //cylinder1.set(true);
    }

    public void solenoidBack() {
        cylinder.set(DoubleSolenoid.Value.kReverse);
        //cylinder1.set(true);
    }

    public void solenoidOff() {
        cylinder.set(DoubleSolenoid.Value.kOff);
        //cylinder1.set(true);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new PneumaticsCommand(this));

    }

}
