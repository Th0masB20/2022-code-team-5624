// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonomousSubsystem extends SubsystemBase {
  TurretSubsystem turretSub;
  DriveSubsystem driveSub;
  VisionSubsystem visionSub;
  Timer timer;
  double driveFor = 3;
  double stopFor = 2;
  boolean startRotation = false;
  double speed = 0.6;
  boolean reset = false;
  
  /** Creates a new AutonomousSubsystem. */
  public AutonomousSubsystem(TurretSubsystem tSub, DriveSubsystem dSub) {
    turretSub = tSub;
    driveSub = dSub;
    visionSub = new VisionSubsystem();
    timer = new Timer();
  }

  public void runAutonomous(){
    if(timer.get() < driveFor){
      driveSub.drive(speed, speed);
    }
    else {
      driveSub.stop();
    }

    /*
    else{
      turretSub.autonomousTurret();
    }
    
    else {
      startRotation = true;
    }
    if(startRotation){
      driveSub.rotateUsingCamera(speed, visionSub.getTx());
    }

    if(driveSub.facingTarget(visionSub.getTx()) && !visionSub.collectedBall()){
      if(!reset){
        driveSub.resetGyro();
        reset = true;
      }
      driveSub.driveStraight(0.05,0);
    }
    
    if(visionSub.collectedBall()){
      turretSub.autonomousTurret();
    }
    */
  }

  public void testTurret(){
    turretSub.shootMotor1.set(0.1);
  }
  public void start(){
    timer.start();
    driveSub.resetGyro();
  }
}
