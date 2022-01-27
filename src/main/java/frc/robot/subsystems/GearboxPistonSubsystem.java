// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GearboxPistonSubsystem extends SubsystemBase {
  Solenoid solenoid1;
  Solenoid solenoid2;
  int counter = 0;
  long timer = 0;
  /** Creates a new GearboxPistonSubsystem. */
  public GearboxPistonSubsystem() {
    solenoid1 = new Solenoid(PneumaticsModuleType.REVPH,Constants.solenoidPort1);
    solenoid2 = new Solenoid(PneumaticsModuleType.REVPH,Constants.solenoidPort2);
  }

  public void UsePistons(boolean activate) {
    if(activate && counter == 0 && cooldown(timer)) {
      solenoid1.set(true);
      solenoid2.set(true);
      timer = System.currentTimeMillis();
      counter++;
      activate = false;
    }
    if(activate && counter == 1 && cooldown(timer)) {
      solenoid1.set(false);
      solenoid2.set(false);
      timer = System.currentTimeMillis();
      counter--;
      activate = false;
    }
  }
  public boolean cooldown(long startTime) {
    long cooldownTime = System.currentTimeMillis()-startTime;
    return (cooldownTime > 250);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
