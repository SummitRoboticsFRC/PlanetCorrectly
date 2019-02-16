/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;
import org.usfirst.frc.team7707.robot.commands.VisionCommand;
/**
 * Add your docs here.
 */
public class VisionSubsystem extends Subsystem {
  NetworkTable table; 
  NetworkTableEntry tx; 
  NetworkTableEntry ty;
  final double CAMERA_Y_ANGLE=0.0;  
  final double TARGET_HEIGHT = 79.0956;
  double cameraHeight = 58.5; //temporary height
  double x; 
  double y; 
  double distanceToTarg; 
  public VisionSubsystem(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); 
    tx = table.getEntry("ty"); 
    UpdateValues(); 
  }
  public void makePath(){
    UpdateValues();
    double e = CAMERA_Y_ANGLE+this.y; 
    this.distanceToTarg = (TARGET_HEIGHT-cameraHeight)/Math.tan(e*180/Math.PI);
  }
  public void UpdateValues(){
    this.x = tx.getDouble(1.0);
    this.y = ty.getDouble(1.0);
  }
  public void PostToDashBoard(){
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightX", x); 
    SmartDashboard.putNumber("Target Distance", distanceToTarg); 
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VisionCommand(this));
  }
}
