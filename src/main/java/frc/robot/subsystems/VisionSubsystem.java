// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  NetworkTable limelightTable;
  double cameraAngle = 22.0;//30; //degress 
  double cameraHeight = (double)25/12; //0.4064; //m
  double height = (double)38.25/12; //2.64; //m
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }
  public double getTx() {
    return limelightTable.getEntry("tx").getDouble(0.0);
  }
  public double getTy() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getDistance() {
    return (height-cameraHeight) / Math.tan(Math.toRadians(cameraAngle + getTy()));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
