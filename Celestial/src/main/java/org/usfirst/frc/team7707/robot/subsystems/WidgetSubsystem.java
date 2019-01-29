/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.commands.DefaultWidgetCommand;

/**
 * Add your docs here.
 */
public class WidgetSubsystem extends Subsystem {
  SpeedController controller;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
/**
     * Widget Subsystem. The constructor for a widget needs to know the motor controller that
     * is connected to the widget device.
     * 
     * @param controller - The motor controller that moves the widget device
     */
    public WidgetSubsystem(SpeedController controller) {
    	this.controller = controller;
    	this.controller.set(0);		// and start with the motor stopped
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
     * setMoveUp - Cause the widget to start moving upwards.
     * When the appropriate movement has been completed setMoveStop must be called
     */
    public void setMoveUp() {
    	controller.set(RobotMap.WIDGET_SPEED_UP);
    }

    /**
     * setMoveDown - Cause the widget to start moving downwards.
     * When the appropriate movement has been completed setMoveStop must be called
     */
    public void setMoveDown() {
    	controller.set(RobotMap.WIDGET_SPEED_DOWN);
    }

    /**
     * setMoveStop - Causes the widget to stop moving
     */
    public void setMoveStop() {
    	controller.set(0);
    }
    
    // Put methods for talking about this subsystem here
    // call these when other things need to know about this subsystem

    /**
     * isInPosition identifies whether the widget is in position. In the demo case this
     * is a dummy method that always returns true. On a real robot this method would check a sensor
     * to determine if the widget was in the correct position
     * 
     * @return true if the widget is in the correct position
     */
    public boolean isInPosition() {
    	return true;
    }
    
    /**
     * Default command. Should not be called directly.
     */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new DefaultWidgetCommand(this));
    }
}
