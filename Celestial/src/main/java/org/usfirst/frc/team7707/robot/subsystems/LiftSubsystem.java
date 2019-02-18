/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team7707.robot.Robot;
import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.RobotMap.LiftStatus;
import org.usfirst.frc.team7707.robot.commands.DefaultLiftCommand;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSupplier speed;
  private BooleanSupplier buttonL1, buttonL2, buttonL3;
  private double kP, kI, kD, period, level1, level2, level3, min, max; 
  //,Vm, Vi, Ri;
  private SpeedController motor;
  private AnalogInput ultrasonic;
  private PIDController heightController;
  private PIDSource heightSource;
  private PIDSourceType sourceType;
  private boolean enabled;

  public LiftSubsystem(DoubleSupplier speed, SpeedController motor, AnalogInput ultrasonic, BooleanSupplier buttonL1, BooleanSupplier buttonL2, BooleanSupplier buttonL3) {

    this.speed = speed;
    this.motor = motor;
    this.ultrasonic = ultrasonic;
    this.buttonL1 = buttonL1;
    this.buttonL2 = buttonL2;
    this.buttonL3 = buttonL3;

    //john
    this.kP = SmartDashboard.getNumber("Lift kP", 0.1);
    this.kI = SmartDashboard.getNumber("Lift kI", 0.1);
    this.kD = SmartDashboard.getNumber("Lift kD", 0.1);
    this.period = SmartDashboard.getNumber("Lift Period", 10);
    
    this.level1 = SmartDashboard.getNumber("Level 1 Height (V)", 2);
    this.level2 = SmartDashboard.getNumber("Level 2 Height (V)", 2);
    this.level3 = SmartDashboard.getNumber("Level 3 Height (V)", 2);
    this.min = SmartDashboard.getNumber("Min Lift Height (cm)", 0);
    this.max = SmartDashboard.getNumber("Max Lift Height (cm)", 2);
    // this.sourceType = PIDSourceType.kDisplacement;

    //john
    //this.Vm = ultrasonic.getVoltage();
    //this.Vi = 5 / 1024;

    // this.heightSource = new PIDSource() {
    
    //   @Override
    //   public void setPIDSourceType(PIDSourceType pidSource) {
        
    //   }
    
    //   @Override
    //   public double pidGet() {
    //     //Vm = ultrasonic.getVoltage();
    //     //Ri = 0.5 * (Vm / Vi);  
    //     return  0.5 * 1024 * ultrasonic.getVoltage() / 5;
    //   }
    
    //   @Override
    //   public PIDSourceType getPIDSourceType() {
    //     return null;
    //   }
    // };
    
    //this.heightSource.setPIDSourceType(sourceType);
    
    this.heightController = new PIDController(kP, kI, kD, ultrasonic, motor);
    this.heightController.setOutputRange(-0.2, 0.3);

    this.enabled = false;
  }

  //john
  public double height() {
    //Vm = ultrasonic.getVoltage();
    //Ri = 0.5 * (Vm / Vi);  
    //return ultrasonic.getRangeInches();
    //return  0.5 * ultrasonic.getVoltage() / 1024.0;
    return 0.5 * ultrasonic.getVoltage() / 0.004883;
  }

  public boolean inRange() {
    //return height() > min && height() < max;
    return true;
  }

  //john
  public void updateValues() {
    kP = SmartDashboard.getNumber("Lift kP", 0.1);
    kI = SmartDashboard.getNumber("Lift kI", 0.1);
    kD = SmartDashboard.getNumber("Lift kD", 0.1);
    period = SmartDashboard.getNumber("Lift Period", 10);

    //SmartDashboard.putNumber("Lift Height (cm)", height());
    
    level1 = SmartDashboard.getNumber("Level 1 Height (cm)", 10);
    level2 = SmartDashboard.getNumber("Level 2 Height (cm)", 10);
    level3 = SmartDashboard.getNumber("Level 3 Height (cm)", 10);
    min = SmartDashboard.getNumber("Min Lift Height (cm)", 0);
    max = SmartDashboard.getNumber("Max Lift Height (cm)", 36);
  }

  public void lift(RobotMap.LiftStatus status) {

    switch (status){
      case NO_COMMAND:
      default:
        motor.set(0.0);
        break;
      case LIFT_MANUAL:
      //default:
       // while(inRange()&&(speed.getAsDouble() < -0.1 || speed.getAsDouble() > 0.1)){
         // System.out.println("SCHMOOP");
          if (speed.getAsDouble() < -0.1) {
            motor.set(0.3);
          } else if (speed.getAsDouble() > 0.1) {
            motor.set(-0.2);
          } else {
            motor.set(0.0);
          }
         // motor.set(0.0);
        //}
        //System.out.println("OUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUT ****************");
        break;

      //john

      /* case LIFT_LEVEL_1:
        heightController.setSetpoint(level1);
        heightController.enable();
        break;

      case LIFT_LEVEL_2:
        heightController.setSetpoint(level2);
        heightController.enable();
        break;

      case LIFT_LEVEL_3:
        heightController.setSetpoint(level3);
        heightController.enable();
        break; */

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
    //john
    setDefaultCommand(new DefaultLiftCommand(this, speed, buttonL1, buttonL2, buttonL3));
  }
}
