/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;
import org.usfirst.frc.team7707.robot.commands.VisionCommand;
import org.usfirst.frc.team7707.robot.RobotMap;
/**
 * Add your docs here.
 */
public class VisionSubsystem extends Subsystem {
  NetworkTable table; 
  NetworkTableEntry tx; 
  NetworkTableEntry ty;
  final double CAMERA_Y_ANGLE=0.0;  
  final double TARGET_HEIGHT = 79.0956;
  final double TARGET_WIDTH = 36.5;
  private double cameraHeight;
  private double xTop; 
  private double yTop; 
  private double xSide; 
  private double ySide;
  private double distanceToTarg; 
  private double angleToTarg;
  AnalogInput ultrasonic;
  public double firstTurn;
  public double driveDist;

  public VisionSubsystem(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); 
    ty = table.getEntry("ty"); 
    ultrasonic = new AnalogInput(RobotMap.ultrasonicInput);
    UpdateValues(); 
  }
  public void makePath(){
    UpdateValues();
    double eC = CAMERA_Y_ANGLE+this.yTop; 
    double eS = CAMERA_Y_ANGLE+this.ySide; 
    angleToTarg=this.xTop;
    double angle_center_side = xTop-xSide; 
    this.distanceToTarg = (TARGET_HEIGHT-cameraHeight)/Math.tan(eC*(Math.PI/180));
    double distanceToTargSide = (TARGET_HEIGHT-cameraHeight)/Math.tan(eS*(Math.PI/180));
    double angle_center_wall; 
      angle_center_wall = Math.asin( (Math.sin((angle_center_side/2)*(Math.PI/180))*distanceToTargSide) / (TARGET_WIDTH/2))*(180/Math.PI); 
    double angle_side_wall; 
      angle_side_wall = 180-angle_center_wall-angle_center_side; 
    double h = (TARGET_WIDTH/2)/(Math.cos(angle_side_wall)*(Math.PI/180));
    double k = (TARGET_WIDTH/2)/(Math.sin(angle_side_wall)*(Math.PI/180));
    double distanceToTarg_Y = ((distanceToTargSide-h)*k)/h+k;
    double distanceToTarg_Ang = Math.asin(distanceToTarg_Y/distanceToTarg)*(180/Math.PI); 
    double distanceToTarg_Yhalf = Math.asin((distanceToTarg_Y/2)/distanceToTarg)*(180/Math.PI); 
      firstTurn = angleToTarg+(distanceToTarg_Ang-distanceToTarg_Yhalf); 
    double distanceToTarg_X = Math.sqrt(distanceToTarg*distanceToTarg-distanceToTarg_Y*distanceToTarg_Y); 
      driveDist = Math.sqrt(distanceToTarg_X*distanceToTarg_X+Math.pow(distanceToTarg_Y/2, 2)); 
      
  }
  public void UpdateValues(){
    table.getEntry("pipeline").setNumber(0); //top side
    this.xTop = tx.getDouble(1.0);
    this.yTop = ty.getDouble(1.0);
    if(this.xTop>0){
      table.getEntry("pipeline").setNumber(1); //Right side
    }else{
      table.getEntry("pipeline").setNumber(2); //Left side
    }
    this.xSide = tx.getDouble(1.0);
    this.ySide = ty.getDouble(1.0);
    table.getEntry("pipeline").setNumber(0); //top side
    double e = CAMERA_Y_ANGLE+this.yTop; 
    this.distanceToTarg = (TARGET_HEIGHT-cameraHeight)/Math.tan(e*180/Math.PI);

    
    cameraHeight = 0.5 * ultrasonic.getVoltage() / 1024.0  + 20.0;
  }

  public void PostToDashBoard(){
    SmartDashboard.putNumber("LimelightY", yTop);
    SmartDashboard.putNumber("LimelightX", xTop); 
    SmartDashboard.putNumber("Target Distance", distanceToTarg); 
    SmartDashboard.putNumber("FirstTurn", firstTurn); 
    SmartDashboard.putNumber("Drive Distance", driveDist);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionCommand(this));
  }
}
