// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutonomousSubsystem extends SubsystemBase {
  VictorSP shootMotor1;
  VictorSP shootMotor2;

  Encoder encoder1r;
  Encoder encoder2l; 
  /** Creates a new AutonomousSubsystem. */
  public AutonomousSubsystem() {
    shootMotor1 = new VictorSP(Constants.shooterMotor1);
    shootMotor2 = new VictorSP(Constants.shooterMotor2);

    encoder1r = new Encoder(Constants.encoderRportA, Constants.encoderRportB);
    encoder2l = new Encoder(Constants.encoderLportA, Constants.encoderLportB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
