// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorSubsystem extends SubsystemBase {
  VictorSP intakeMotor;
  VictorSP beltMotor;
  VictorSP climbMotor1,climbMotor2;
  /** Creates a new MotorIntakeSubsystem. */
  public MotorSubsystem() {
    intakeMotor = new VictorSP(Constants.intakeMotorPort);
    beltMotor = new VictorSP(Constants.beltMotorPort);
    climbMotor1 = new VictorSP(Constants.pistonMotorPort1);
    climbMotor2 = new VictorSP(Constants.pistonMotorPort2);
  }

  public void useIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public void useBeltMotor(double speed) {
    beltMotor.set(speed);
  }
  
  public void usePistonMotors(double speed) {
    climbMotor1.set(speed);
    climbMotor2.set(speed);

  }

  public void stopIntakeMotors(){
    intakeMotor.set(0);
  }

  public void stopClimbMotors(){
    climbMotor1.set(0);
    climbMotor2.set(0);
  }

  public void stopBeltMotors(){
    beltMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
