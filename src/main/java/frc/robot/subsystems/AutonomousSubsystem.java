// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class AutonomousSubsystem extends SubsystemBase {
  WPI_TalonFX falcon1;
  WPI_TalonFX falcon2; 
  VisionSubsystem vision;

  double p = 0,i = 0,d = 0;
  double kp, ki;

  /** Creates a new AutonomousSubsystem. */
  public AutonomousSubsystem() {
    falcon1 = new WPI_TalonFX(1);
    falcon2 = new WPI_TalonFX(2);

    falcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  private double pid(double error) {
    i += error * 0.02f;
    double value = kp*error + ki*i;
    return value;
  }
  public void measureDistance(){

  }

  public double getVelocity1(){
    return falcon1.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
