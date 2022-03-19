// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID; 

//two encoders

public class DriveSubsystem extends SubsystemBase {
  private Spark motorR1;
  private Spark motorR2;

  private Spark motorL1;
  private Spark motorL2;

  AHRS gyro;
  PID rotatePid, drivePid;
  double kp, ki;
  double kpDrive, kiDrive;
  
  private MotorControllerGroup rightGroup;
  private MotorControllerGroup leftGroup;

  double error;
  double value;
  double rightSpeed;
  double leftSpeed;

  Timer timer;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
    motorR1 = new Spark(Constants.CANPortR1);
    motorR2 = new Spark(Constants.CANPortR2);

    motorL1 = new Spark(Constants.CANPortL1);
    motorL2 = new Spark(Constants.CANPortL2);
    
    rightGroup = new MotorControllerGroup(motorR1, motorR2);
    leftGroup = new MotorControllerGroup(motorL1, motorL2);

    gyro = new AHRS(SPI.Port.kMXP);
    rotatePid = new PID(kp, ki);
    drivePid = new PID(kpDrive, kiDrive);
    timer = new Timer();
  }


  public void drive(double leftY, double rightY) 
  {
    if(rightY > 0.03 || rightY < -0.03 || leftY > 0.03 || leftY < -0.03){
      rightGroup.set(Math.abs(rightY) * rightY);
      leftGroup.set(Math.abs(leftY) * leftY);
    }
    else{
      stop();
    }
  }

  public void driveStraight(double speed, double target)
  {
    error = target - gyro.getAngle();
    value = drivePid.calculatePid(error);
    rightSpeed = speed + value;
    leftSpeed = speed - value;

    drive(speed, speed);

    SmartDashboard.putNumber("angle difference", error);
    SmartDashboard.putNumber("speed left", leftSpeed);
    SmartDashboard.putNumber("speed right", rightSpeed);
  }

  public void rotateToTarget(double speed, double target)
  {
    error = target - gyro.getAngle();
    value = rotatePid.calculatePid(error);
    rightSpeed = speed + value;
    leftSpeed = speed - value;

    if((rightSpeed > 0.5 || leftSpeed > 0.5) || (rightSpeed < -0.5 || leftSpeed < -0.5))
    {
       rightGroup.set(0.5);
       leftGroup.set(0.5);
    }
    else
    {
      rightGroup.set(rightSpeed);
      leftGroup.set(leftSpeed);
    }
  }

  public void rotateUsingCamera(double speed, double error)
  {
    value = rotatePid.calculatePid(error);
    rightSpeed = speed + value;
    leftSpeed = speed - value;

    if((rightSpeed > 0.5 || leftSpeed > 0.5) || (rightSpeed < -0.5 || leftSpeed < -0.5))
    {
       rightGroup.set(0.5);
       leftGroup.set(0.5);
    }
    else
    {
      rightGroup.set(rightSpeed);
      leftGroup.set(leftSpeed);
    }
  }


  public boolean facingTarget(double error)
  {
    if(error == 0)
    {
      timer.start();
      if(timer.get() > 0.5)
      {
        return true;
      }
    }
    else
    {
      timer.stop();
      timer.reset();
    }
    return false;
  }

  public void stop()
  {
    rightGroup.set(0);
    leftGroup.set(0);
  }

  public void resetGyro(){
    gyro.reset();
  }
}
