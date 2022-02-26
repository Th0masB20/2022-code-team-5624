// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;

public class TurretSubsystem extends SubsystemBase {
  
  VictorSP shootMotor1;
  VictorSP shootMotor2; 
  VictorSP rotateMotor;
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
    shootMotor1 = new VictorSP(Constants.shootPort1);
    shootMotor2 = new VictorSP(Constants.shootPort2);
    rotateMotor = new VictorSP(Constants.turretRotatePort);;
    shootPid = new PID(kp,ki,kd);
    turretPID = new PID(kp,ki);

    vision = new VisionSubsystem();
  }

  public void turnTurret() {
    if (vision.getTx()!=0||vision.getTy()!=0){
      rotateMotor.set(turretPID.calculatePid(vision.getTx()));
    } else {
      turretStop();
    }
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
    
    SmartDashboard.putNumber("speed",turretPID.calculatePid(vision.getTx()) );
  }

  public void manualTurret(double turn){
    rotateMotor.set(turn);
  }
  public void turretStop() {
    rotateMotor.set(0);
  }
  public void runTurret(double distance) {
    //wantedSpeed =;
    error = wantedSpeed - (/*encoder velocity*/  convertionFactor);
    shootMotor1.set(shootPid.calculatePid(error)); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
