/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team7707.robot.commands.RatchetCommand;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class RatchetSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSupplier backSpeed, frontSpeed;
  private SpeedController backMotor, frontMotor;
  private Joystick driverInput;
  private int backDescendButton, frontDescendButton;
  
  public boolean doBackDescend, doFrontDescend;
  
  private boolean enabled;

  public RatchetSubsystem(DoubleSupplier backSpeed, DoubleSupplier frontSpeed, SpeedController backMotor, SpeedController frontMotor, Joystick driverInput, int backDescendButton, int frontDescendButton) {
    this.backSpeed = backSpeed;
    this.frontSpeed = frontSpeed;
    this.backMotor = backMotor;
    this.frontMotor = frontMotor;
    this.driverInput = driverInput;
    this.backDescendButton = frontDescendButton;
    this.frontDescendButton = backDescendButton;
    this.doBackDescend = false;
    this.doFrontDescend = false;
    this.enabled = false;
  }

  public void lift() {
    double back = backSpeed.getAsDouble();
    double front = frontSpeed.getAsDouble();

    if (back > 0.5) {
      backMotor.set(1.0);
    //  frontMotor.set(1.0);
    } else if (front > 0.5) {
    //  backMotor.set(-1.0);
        frontMotor.set(1.0);
    //  frontMotor.set(-1.0);
    } else {
    //  backMotor.set(0);
      frontMotor.set(0);
    }
    
    /* backMotor.set(backSpeed.getAsDouble());
    frontMotor.set(frontSpeed.getAsDouble()); */
  }

  public void backDescend() {
    backMotor.set(-1.0);
  }
  
  public void frontDescend() {
    frontMotor.set(-1.0);
  }

  public void getButtons() {
    this.doFrontDescend = driverInput.getRawButton(backDescendButton);
    this.doBackDescend = driverInput.getRawButton(frontDescendButton);
  }

  public RatchetSubsystem setEnabled(boolean enabled) {
    this.enabled = enabled;
    return this;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RatchetCommand(this));
  }
}
