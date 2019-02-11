/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team7707.robot.commands.DefaultLiftCommand;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSupplier speed;
  private SpeedController motor;
  private Ultrasonic ultrasonic;
  private boolean enabled;

  public LiftSubsystem(DoubleSupplier speed, SpeedController motor) {
    this.speed = speed;
    this.motor = motor;
    this.ultrasonic = ultrasonic;
    this.enabled = false;
  }

  public void lift() {
    motor.set(speed.getAsDouble());
  }

  public double getHeight() {
    return 1.3; //INFO: fix this
  }

  public LiftSubsystem setEnabled(boolean enabled) {
    this.enabled = enabled;
    return this;
  }

    @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultLiftCommand(this));
  }
}
