// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends CommandBase {

  AutonomousSubsystem autoSub;
  DriveSubsystem driveSub;

  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(AutonomousSubsystem autoSub/* DriveSubsystem driveSub*/) {
    this.autoSub = autoSub;
    //this.driveSub = driveSub;
    //addRequirements(autoSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //autoSub.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //autoSub.runAutonomous();
    //driveSub.driveStraight(0.3, 0);
    autoSub.testTurret();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

