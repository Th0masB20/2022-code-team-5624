// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PID;

public class TurretSubsystem extends SubsystemBase {
  WPI_TalonFX falcon1;
  WPI_TalonFX falcon2; 
  WPI_TalonFX rotateMotor;
  VisionSubsystem vision;

  double kp = 0.1;
  double ki;
  double kd;
  double wantedSpeed = 0;
  double error = 0;
  PID turretPID;
  PID shootPid;
  Timer timer = new Timer();
  double radius = 0;
  double convertionFactor = (double)(1/2048) * (2 * Math.PI * radius);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    falcon1 = new WPI_TalonFX(1);
    falcon2 = new WPI_TalonFX(2);
    rotateMotor = new WPI_TalonFX(3);
    shootPid = new PID(kp,ki,kd);
    turretPID = new PID(kp,ki);
    falcon2.follow(falcon1);
    falcon2.setInverted(true);

    vision = new VisionSubsystem();

    falcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    falcon2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void turnTurret() {
    rotateMotor.set(turretPID.calculatePid(vision.getTx()));
    if (vision.getTx()==0)

      timer.start();
    else 
    {
    timer.stop();
    timer.reset();
    }
  }

  public void autonomousTurret () {
    turnTurret();
    if (timer.get() > 0.69){
      runTurret(vision.getDistance());
    }
    
    SmartDashboard.putNumber("getTy", vision.getTy());
    SmartDashboard.putNumber("distance", vision.getDistance());
  }

  public void runTurret(double distance) {
    //wantedSpeed =;
    error = wantedSpeed - (falcon1.getSelectedSensorVelocity() * convertionFactor);
    falcon1.set(shootPid.calculatePid(error));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
