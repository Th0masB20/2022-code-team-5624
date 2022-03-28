// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;

//two encoders

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax motorR1;
  private CANSparkMax motorR2;

  private CANSparkMax motorL1;
  private CANSparkMax motorL2;

  private AHRS gyro;
  PID rotatePid, drivePid;
  double kp = 0.1, ki = 0;
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
    motorR1 = new CANSparkMax(Constants.CANPortR1, MotorType.kBrushless);
    motorR2 = new CANSparkMax(Constants.CANPortR2, MotorType.kBrushless);

    motorL1 = new CANSparkMax(Constants.CANPortL1, MotorType.kBrushless);
    motorL2 = new CANSparkMax(Constants.CANPortL2, MotorType.kBrushless);
    
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
      rightGroup.set(rightY * 0.5);
      leftGroup.set(leftY * 0.5);
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

    if((rightSpeed < 0.4 || leftSpeed < 0.4) || (rightSpeed > -0.4 || leftSpeed > -0.4))
    {
      rightGroup.set(rightSpeed);
      leftGroup.set(leftSpeed);
    }
    
    SmartDashboard.putNumber("angle difference", error);
    SmartDashboard.putNumber("speed left", leftSpeed);
    SmartDashboard.putNumber("speed right", rightSpeed);
  }

  public void rotateToTarget(double target)
  {
    error = target - gyro.getAngle();
    value = rotatePid.calculatePid(error);
    rightSpeed = value;
    leftSpeed = value;

    if((rightSpeed > 0.4 || leftSpeed > 0.4) || (rightSpeed < -0.4 || leftSpeed < -0.4))
    {
       rightGroup.set(0.4);
       leftGroup.set(0.4);
    }
    else
    {
      rightGroup.set(rightSpeed);
      leftGroup.set(leftSpeed);
    }
  }

  public void stop()
  {
    rightGroup.set(0);
    leftGroup.set(0);
  }

  public void resetGyro(){
    gyro.reset();
  }

  public AHRS getGyro(){
     return gyro;
  }
}
