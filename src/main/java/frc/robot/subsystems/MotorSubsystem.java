// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorSubsystem extends SubsystemBase {
  VictorSPX intakeMotor;
  VictorSPX beltMotor;
  VictorSPX climbMotor1,climbMotor2;
  /** Creates a new MotorIntakeSubsystem. */
  public MotorSubsystem() {
    intakeMotor = new VictorSPX(Constants.intakeMotorPort);

    climbMotor1 = new VictorSPX(Constants.pistonMotorPort1);
    climbMotor2 = new VictorSPX(Constants.pistonMotorPort2);
  }

  //intake motor
  public void useIntakeMotor(double outSpeed, double inSpeed) {
    if(outSpeed > 0){
      intakeMotor.set(VictorSPXControlMode.PercentOutput,-outSpeed);
    }
    else if(inSpeed > 0){
      intakeMotor.set(VictorSPXControlMode.PercentOutput,inSpeed);
    }
    else {
      stopIntakeMotors();
    }
  }

  //climb motor
  public void useClimbMotors(double speed) {
    if (speed<-0.2||speed>0.2) 
    {
      climbMotor1.set(VictorSPXControlMode.PercentOutput, speed);
      climbMotor2.set(VictorSPXControlMode.PercentOutput, speed);
    } else {
      stopClimbMotors();
    }
  }

  public void stopIntakeMotors(){
    intakeMotor.set(VictorSPXControlMode.PercentOutput,0);
  }

  public void stopClimbMotors(){
    climbMotor1.set(VictorSPXControlMode.PercentOutput, 0);
    climbMotor2.set(VictorSPXControlMode.PercentOutput, 0);
  }
}
