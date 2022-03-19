// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class MotorCommand extends CommandBase {
  MotorSubsystem motorSub;
  TurretSubsystem turretSub;
  boolean start = false;

  /** Creates a new IntakeBeltCommand. */

  public MotorCommand(MotorSubsystem motorSub,TurretSubsystem turretSub) {
    this.motorSub = motorSub;
    this.turretSub = turretSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.motorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //motorSub.useBeltMotor(RobotContainer.xboxController.getLeftY());
    motorSub.useClimbMotors(RobotContainer.xboxController.getLeftX());

    motorSub.useIntakeMotor(RobotContainer.xboxController.getLeftTriggerAxis(), RobotContainer.xboxController.getRightTriggerAxis());
    //turretSub.manualTurret(RobotContainer.xboxController.getLeftX()); // change later

    if(RobotContainer.xboxController.getStartButton()){
      start = true;
    }
    if(RobotContainer.xboxController.getBackButton()){
      start = false;
    }

    if(start){
      turretSub.autonomousTurret();
    }
    else {
      turretSub.turretStop();
    }
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
