// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePistonSubsystem extends SubsystemBase {
  DoubleSolenoid solenoid;
  int counter = 0; 
  long start = 0;
  /** Creates a new IntakePistonSubsystem. */
  public IntakePistonSubsystem() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,Constants.intakeSolonoidPort1, Constants.intakeSolonoidPort2);
  
  }
  public void intakePistons ( XboxController controller ) {
    if(controller.getRawButton(1) && counter == 0 && waitTime(start)) {
      solenoid.set(Value.kForward); 
      counter++;
      start = System.currentTimeMillis();
    } 
    if(controller.getRawButton(1) && counter == 1 && waitTime(start))
    solenoid.set(Value.kReverse);
    counter--;
    start = System.currentTimeMillis();
    
  }

  private boolean waitTime (long start) {
    if (System.currentTimeMillis() - start > 250 ) 
    {
      return true;
    }
    return false;

  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
