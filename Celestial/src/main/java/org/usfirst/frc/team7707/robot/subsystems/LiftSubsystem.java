/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import java.util.function.DoubleSupplier;
//import java.util.function.BooleanSupplier;

import org.usfirst.frc.team7707.robot.Robot;
import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.commands.DefaultLiftCommand;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.dynalink.beans.StaticClass;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;

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
  //private BooleanSupplier buttonL1, buttonL2, buttonL3;

  public double kP, kI, kD, period, liftL1, liftL2, liftL3, liftMin, liftMax;

  private PIDController heightControl;

  public LiftSubsystem(DoubleSupplier speed, SpeedController motor, Ultrasonic ultrasonic) {
    this.speed = speed;
    this.motor = motor;
    this.ultrasonic = ultrasonic;

    // this.buttonL1 = buttonL1;
    // this.buttonL2 = buttonL2;
    // this.buttonL3 = buttonL3;

    this.kP = SmartDashboard.getNumber("Lift kP", 0.1);
    this.kI = SmartDashboard.getNumber("Lift kI", 0.1);
    this.kD = SmartDashboard.getNumber("Lift kD", 0.1);
    this.period = SmartDashboard.getNumber("Lift Period", 10);

    this.liftL1 = SmartDashboard.getNumber("Level 1 Height (inches)", 10);
    this.liftL2 = SmartDashboard.getNumber("Level 2 Height (inches)", 10);
    this.liftL3 = SmartDashboard.getNumber("Level 3 Height (inches)", 10);
    this.liftMin = SmartDashboard.getNumber("Min Lift Height (inches)", 0);
    this.liftMax = SmartDashboard.getNumber("Max Lift Height (inches)", 36);

    this.heightControl = new PIDController(kP, kI, kD, ultrasonic, motor);

    this.enabled = false;
  }

  public double height() {
    //return ultrasonic.getRangeInches();
    return SmartDashboard.getNumber("Lift Height (inches)", ultrasonic.getRangeInches());
  }

  public boolean inRange() {
    return height() > liftMin && height() < liftMax;
  }

  // public int getButton() {
  //   int buttonNum = 0;
  //   if(buttonL1.getAsBoolean()) {
  //     buttonNum = 1;
  //   }
  //   else if(buttonL2.getAsBoolean()) {
  //     buttonNum = 2;
  //   }
  //   else if(buttonL3.getAsBoolean()) {
  //     buttonNum = 3;
  //   }
  //   return buttonNum;
  // }

  public void lift(RobotMap.LiftStatus liftStatus) {
    switch(liftStatus) {
      case LIFT_MANUAL:
        while(inRange()) {
          motor.set(speed.getAsDouble());
        }
        break;
      case LIFT_LEVEL_1:
        heightControl.enable();
        heightControl.setSetpoint(liftL1);
        break;
      case LIFT_LEVEL_2:
        heightControl.enable();
        heightControl.setSetpoint(liftL2);
        break;
      case LIFT_LEVEL_3:
        heightControl.enable();
        heightControl.setSetpoint(liftL3);
        break;
    }
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
