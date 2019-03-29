/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.lang.Math;
import org.usfirst.frc.team7707.robot.commands.VisionCommand;
/**
 * Add your docs here.
 */
public class VisionSubsystem extends Subsystem implements PIDSource{
  NetworkTable table; 
  NetworkTableEntry tx; 
  NetworkTableEntry ty;
  final double CAMERA_Y_ANGLE=0.0;  
  final double TARGET_HEIGHT = 79.0956;
  final double TARGET_HEIGHT_MID = 73.5;
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
  private PIDSourceType pid;

  public VisionSubsystem(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); 
    ty = table.getEntry("ty"); 
    // ultrasonic = new AnalogInput(RobotMap.ultrasonicInput); // causes error 
    //this.ultrasonic = ultrasonic;
    UpdateValues();
    setPIDSourceType(pid);
  }

  public void makePath() {
    UpdateValues();
    double eC, eS, h, k;
    double angle_center_side, angle_center_wall, angle_side_wall;
    double distanceToTargSide, distanceToTarg_Ang;
    double distanceToTarg_X, distanceToTarg_Y, distanceToTarg_Yhalf;

    eC = 0.0 + this.yTop;     // camera y angle = 0
    eS = 0.0 + this.ySide;    // target height = 79.0956, mid = 73.5 (below)
    angleToTarg = this.xTop;  // camera height = 61.0 (below)
    distanceToTarg = (79.0956 - 61.0) / Math.tan(eC * (Math.PI/180));
    distanceToTargSide = (73.5 - 61.0) / Math.tan(eS * (Math.PI/180));

    angle_center_side = xTop - xSide;   // target width = 36.5 (below)
    angle_center_wall = Math.asin((Math.sin((angle_center_side / 2) * (Math.PI / 180)) * distanceToTargSide) / (36.5/2)) * (180 / Math.PI); 
    angle_side_wall = 180 - angle_center_wall - angle_center_side; 

    // DO ALL THE ACTUAL MATH ONCE YOU CAN TEST IT

    h = (36.5/2) / (Math.cos(angle_side_wall) * (Math.PI / 180));
    k = (36.5/2) / (Math.sin(angle_side_wall) * (Math.PI / 180));

    distanceToTarg_Y = ((distanceToTargSide - h) * k) / h + k;
    distanceToTarg_Ang = Math.asin(distanceToTarg_Y / distanceToTarg) * (180 / Math.PI); 
    distanceToTarg_Yhalf = Math.asin((distanceToTarg_Y/2) / distanceToTarg) * (180 / Math.PI); 
      firstTurn = angleToTarg + (distanceToTarg_Ang - distanceToTarg_Yhalf); 

    distanceToTarg_X = Math.sqrt(distanceToTarg * distanceToTarg - distanceToTarg_Y * distanceToTarg_Y); 
      driveDist = Math.sqrt(distanceToTarg_X * distanceToTarg_X + Math.pow(distanceToTarg_Y / 2, 2)); 
      
  }


  public void UpdateValues(){
    this.xTop = getPipeLineZero()[0];
    this.yTop = getPipeLineZero()[1];
  }

  public double[] getPipeLineZero(){
    table.getEntry("pipeline").setNumber(0);
    double[] ret = {tx.getDouble(1.0), ty.getDouble(1.0)};
    return ret;
  }

  public double[] getPipeLineOne(){
    table.getEntry("pipeline").setNumber(1);
    double[] ret = {tx.getDouble(1.0), ty.getDouble(1.0)};
    return ret;
  }

  public double[] getPipeLineTwo(){
    table.getEntry("pipeline").setNumber(2);
    double[] ret = {tx.getDouble(1.0), ty.getDouble(1.0)};
    return ret;
  }

  public void PostToDashBoard(){
    //SmartDashboard.putNumber("LimelightY", yTop);
   // SmartDashboard.putNumber("LimelightX", xTop); 
    //SmartDashboard.putNumber("Target Distance", distanceToTarg); 
    SmartDashboard.putNumber("FirstTurn", firstTurn); 
    SmartDashboard.putNumber("Drive Distance", driveDist);
    //SmartDashboard.putNumber("Camera Height (cm)", CAMERA_HEIGHT);
  }

  // implementing PIDSource interface stuff
  public void setPIDSourceType(PIDSourceType pidSource) {
    pid = pidSource.kDisplacement;
  }

  public PIDSourceType getPIDSourceType(){
    return pid;
  }

  public double pidGet(){
    return xTop;
  }

  public double getDistanceToTarg() {
    return distanceToTarg;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionCommand(this));
  }
}
