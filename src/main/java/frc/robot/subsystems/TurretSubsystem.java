// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.ShutdownChannelGroupException;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;


public class TurretSubsystem extends SubsystemBase {
  
  WPI_TalonFX shootMotor1;
  WPI_TalonFX shootMotor2; 
  VictorSP rotateMotor;
  VisionSubsystem vision;

  double kp = 0.1;
  double ki;
  double kd;
  double wantedSpeed = 0;
  double error = 0;
  PID turretPID;
  Timer timer = new Timer();

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    shootMotor1 = new WPI_TalonFX(Constants.shootPort1);
    shootMotor2 = new WPI_TalonFX(Constants.shootPort2);
    rotateMotor = new VictorSP(Constants.turretRotatePort);;

    vision = new VisionSubsystem();
  }

  public void turnTurret() {
    if (vision.getTx()!=0){
      rotateMotor.set(turretPID.calculatePid(vision.getTx()));
    } else {
      turretStop();
    }
    if (vision.getTx()==0)
    {
      timer.start();
    }
    else 
    {
      timer.stop();
      timer.reset();
    }
  }

  public void autonomousTurret () {
    turnTurret();
      if (timer.get() > 0.69){
        shootBall(vision.getDistance());
    }
    shootBall(vision.getDistance());
  }

  public void manualTurret(double turn){
    rotateMotor.set(turn * 0.3);
  }

  public void turretStop() {
    rotateMotor.set(0);
  }

  public void shootBall(double distance) {
    wantedSpeed = -0.3;
    shootMotor1.set(wantedSpeed); 
    shootMotor2.set(wantedSpeed - 0.2);
    SmartDashboard.putNumber("distance", vision.getDistance());
    SmartDashboard.putNumber("wanted speed", wantedSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
