// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private VictorSP motorR1;
  private VictorSP motorR2;
  private VictorSP motorR3;
  private VictorSP motorL1;
  private VictorSP motorL2;
  private VictorSP motorL3;
  
  private MotorControllerGroup rightGroup;
  private MotorControllerGroup leftGroup;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    motorR1 = new VictorSP(Constants.vPortR1);
    motorR2 = new VictorSP(Constants.vPortR2);
    motorR3 = new VictorSP(Constants.vPortR3);
    motorL1 = new VictorSP(Constants.vPortL1);
    motorL2 = new VictorSP(Constants.vPortL2);
    motorL3 = new VictorSP(Constants.vPortL3);

    rightGroup = new MotorControllerGroup(motorR1, motorR2, motorR3);
    leftGroup = new MotorControllerGroup(motorL1, motorL2, motorL3);
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
