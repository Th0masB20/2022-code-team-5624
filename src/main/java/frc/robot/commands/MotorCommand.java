// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MotorSubsystem;

public class MotorCommand extends CommandBase {
  MotorSubsystem motorSub;
  /** Creates a new IntakeBeltCommand. */
  public MotorCommand(MotorSubsystem motorSub) {
    this.motorSub = motorSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.motorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorSub.useBeltMotor(RobotContainer.xboxController.getLeftY());
    motorSub.useIntakeMotor(RobotContainer.xboxController.getRightTriggerAxis());
    motorSub.usePistonMotors(RobotContainer.xboxController.getLeftX());
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
