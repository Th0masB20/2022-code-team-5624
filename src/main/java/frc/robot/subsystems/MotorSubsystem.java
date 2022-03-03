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

  //intake motor
  public void useIntakeMotor(double outSpeed, double inSpeed) {
    if(outSpeed > 0){
      intakeMotor.set(outSpeed);
    }
    else {
      stopIntakeMotors();
    }

    if(inSpeed > 0){
      intakeMotor.set(inSpeed);
    }
    else {
      stopIntakeMotors();
    }
  }

  //belt motor
  public void useBeltMotor(double speed) {
    if (speed<-0.2||speed>0.2) {
      beltMotor.set(speed);
    } else {
      stopBeltMotors();
    }
  }

  //climb motor
  public void useClimbMotors(double speed) {
    if (speed<-0.2||speed>0.2) {
      climbMotor1.set(speed);
      climbMotor2.set(speed);
    } else {
      stopClimbMotors();;
    }
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
}
