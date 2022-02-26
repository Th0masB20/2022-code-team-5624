// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID; 

//two encoders

public class DriveSubsystem extends SubsystemBase {
  private VictorSP motorR1;
  private VictorSP motorR2;

  private VictorSP motorL1;
  private VictorSP motorL2;

  AHRS gyro;
  PID pid;
  double kp, ki;
  
  private MotorControllerGroup rightGroup;
  private MotorControllerGroup leftGroup;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    motorR1 = new VictorSP(1);
    motorR2 = new VictorSP(0);

    motorL1 = new VictorSP(3);
    motorL2 = new VictorSP(2);

    rightGroup = new MotorControllerGroup(motorR1, motorR2);
    leftGroup = new MotorControllerGroup(motorL1, motorL2);

    gyro = new AHRS(SPI.Port.kMXP);
    pid = new PID(kp, ki);
  }

  public void drive(double leftY, double rightY) {
    rightGroup.set(-rightY);
    leftGroup.set(leftY);
  }

  public void driveStraight(double speed){
    double error = 0 - gyro.getAngle();
    //double value = pid.calculatePid(error);
    double s = speed + error/100;
    double sNeg = speed - error/100;

    // if(s > 0.5 || sNeg > 0.5){
    //   if(s < -0.5 || sNeg < -0.5){
    //     rightGroup.set(0);
    //     leftGroup.set(0);
    //   }
    // }
    rightGroup.set(s);
    leftGroup.set(sNeg);

    //rightGroup.set(speed + error/100);
    //leftGroup.set(speed - error/100);
    SmartDashboard.putNumber("angle difference", error);
    SmartDashboard.putNumber("speed left", s);
    SmartDashboard.putNumber("speed right", sNeg);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
