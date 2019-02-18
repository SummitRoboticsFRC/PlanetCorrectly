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

import java.awt.Robot;
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
  private double CAMERA_HEIGHT = 61.0;
  private double xTop; 
  private double yTop; 
  private double xSide; 
  private double ySide;
  private double distanceToTarg; 
  private double angleToTarg;
  //private AnalogInput ultrasonic; //causes error when initializing // 18/02/19 no need for ultrasound
  public double firstTurn;
  public double driveDist;

  public VisionSubsystem(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); 
    ty = table.getEntry("ty"); 
    // ultrasonic = new AnalogInput(RobotMap.ultrasonicInput); // causes error 
    //this.ultrasonic = ultrasonic;
    UpdateValues();
  }
  public void makePath(){
    UpdateValues();
    double eC = CAMERA_Y_ANGLE+this.yTop; 
     // System.out.println("Altitude of Center of Target: "+eC);
    double eS = CAMERA_Y_ANGLE+this.ySide; 
      //System.out.println("Altitide of Side of Target: " + eS);
    angleToTarg=this.xTop;
    double angle_center_side = xTop-xSide; 
      //System.out.println("∆ Angle between center and side: "+angle_center_side);
    this.distanceToTarg = (TARGET_HEIGHT-CAMERA_HEIGHT)/Math.tan(eC*(Math.PI/180));
      //System.out.println("Distance To Target: "+ distanceToTarg);
    double distanceToTargSide = (TARGET_HEIGHT-CAMERA_HEIGHT)/Math.tan(eS*(Math.PI/180));
      //System.out.println("Distance To Side: "+ distanceToTargSide);
    double angle_center_wall; 
      angle_center_wall = Math.asin( (Math.sin((angle_center_side/2)*(Math.PI/180))*distanceToTargSide) / (TARGET_WIDTH/2))*(180/Math.PI); 
      //  System.out.println("Angle of center LOS to wall: "+ angle_center_wall); // should be > 90˚
    double angle_side_wall; 
      angle_side_wall = 180-angle_center_wall-angle_center_side; 
        //System.out.println("Angle of side LOS to wall: "+ angle_side_wall); // should be < 90˚
    double h = (TARGET_WIDTH/2)/(Math.cos(angle_side_wall)*(Math.PI/180));
        //System.out.println("h: "+ h); // should be (+)
    double k = (TARGET_WIDTH/2)/(Math.sin(angle_side_wall)*(Math.PI/180));
        //System.out.println("k: "+ k); //should be (-)
    double distanceToTarg_Y = ((distanceToTargSide-h)*k)/h+k;
        //System.out.println("Vertical Leg of Distance to Target: "+distanceToTarg_Y);
    double distanceToTarg_Ang = Math.asin(distanceToTarg_Y/distanceToTarg)*(180/Math.PI); 
        //System.out.println("Angle of said Triangle: " + distanceToTarg_Ang);
    double distanceToTarg_Yhalf = Math.asin((distanceToTarg_Y/2)/distanceToTarg)*(180/Math.PI); 
        //System.out.println("Distance to a point where Dy is halfed: "+ distanceToTarg_Yhalf);
      firstTurn = angleToTarg+(distanceToTarg_Ang-distanceToTarg_Yhalf); 
        //System.out.println("The first require turn: "+firstTurn);
    double distanceToTarg_X = Math.sqrt(distanceToTarg*distanceToTarg-distanceToTarg_Y*distanceToTarg_Y); 
        //System.out.println("the x component of D: " + distanceToTarg_X);
      driveDist = Math.sqrt(distanceToTarg_X*distanceToTarg_X+Math.pow(distanceToTarg_Y/2, 2)); 
        //System.out.println("the distance that robot will drive: " + driveDist);
      
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
    //double e = CAMERA_Y_ANGLE+this.yTop; 
    //this.distanceToTarg = (TARGET_HEIGHT-CAMERA_HEIGHT)/Math.tan(e*180/Math.PI);

    
   // cameraHeight = 0.5 * ultrasonic.getVoltage() / 0.004883 + 20.0;
  }

  public void PostToDashBoard(){
    SmartDashboard.putNumber("LimelightY", yTop);
    SmartDashboard.putNumber("LimelightX", xTop); 
    SmartDashboard.putNumber("Target Distance", distanceToTarg); 
    SmartDashboard.putNumber("FirstTurn", firstTurn); 
    SmartDashboard.putNumber("Drive Distance", driveDist);
    SmartDashboard.putNumber("Camera Height (cm)", CAMERA_HEIGHT);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionCommand(this));
  }
}
