// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;


public class TurretSubsystem extends SubsystemBase {
  
  WPI_TalonFX shootMotor1; //back
  WPI_TalonFX shootMotor2; //front
  TalonSRX rotateMotor;
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
    shootMotor1 = new WPI_TalonFX(Constants.shootPort1); //can id1
    shootMotor2 = new WPI_TalonFX(Constants.shootPort2); //cam id2
    rotateMotor = new TalonSRX(Constants.turretRotatePort);

    vision = new VisionSubsystem();
  }

  public void turnTurret() {
    if (vision.getTx()!=0){
      rotateMotor.set(TalonSRXControlMode.PercentOutput,-(vision.getTx()/27) * 0.2 /*turretPID.calculatePid(vision.getTx())*/);
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
    SmartDashboard.putNumber("xbox", turn);
      if(turn > 0.1){
        rotateMotor.set(TalonSRXControlMode.PercentOutput,turn);
      }
      else if(turn < -0.1){
        rotateMotor.set(TalonSRXControlMode.PercentOutput,turn);
      }
      else {
        turretStop();
      }

      /*
    if((turnRight) > 0){
      rotateMotor.set(turnRight);
      SmartDashboard.putNumber("turret", rotateMotor.get());
      System.out.println(rotateMotor.get());
    }
    else if(turnLeft > 0){
      rotateMotor.set(turnRight);
      SmartDashboard.putNumber("turret", rotateMotor.get());
      System.out.println(rotateMotor.get());
    } 
    else{
      rotateMotor.set(0);
    }
    */
  }

  public void turretStop() {
    rotateMotor.set(TalonSRXControlMode.PercentOutput,0);
  }

  public void shootBall(double distance) {
    wantedSpeed = -0.2;
    shootMotor1.set(wantedSpeed); //slower 
    shootMotor2.set(wantedSpeed - 0.2); // faster
  }

  public boolean waitTime (long start) {
    if (System.currentTimeMillis() - start > 250 ) 
    {
      return true;
    }
    return false;

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
