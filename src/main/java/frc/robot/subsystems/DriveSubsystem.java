// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  VictorSP motorR1 = new VictorSP(Constants.vPortR1);
  VictorSP motorR2 = new VictorSP(Constants.vPortR2);
  VictorSP motorR3 = new VictorSP(Constants.vPortR3);
  VictorSP motorL1 = new VictorSP(Constants.vPortL1);
  VictorSP motorL2 = new VictorSP(Constants.vPortL2);
  VictorSP motorL3 = new VictorSP(Constants.vPortL3);
  
  
  MotorControllerGroup rightGroup = new MotorControllerGroup(motorR1, motorR2, motorR3);
  MotorControllerGroup leftGroup = new MotorControllerGroup(motorL1, motorL2, motorL3);
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
  }

  public void drive(double leftY, double rightY) {
    rightGroup.set(-rightY);
    leftGroup.set(leftY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
