// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbPistonSubsystem;
import frc.robot.subsystems.GearPistonSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;

public class PistonCommand extends CommandBase {
  ClimbPistonSubsystem climbSub;
  GearPistonSubsystem gearSub;
  IntakePistonSubsystem intakeSub;
  /** Creates a new PistonCommand. */
  public PistonCommand(ClimbPistonSubsystem climbSub, GearPistonSubsystem gearSub, IntakePistonSubsystem intakeSub) {
    this.climbSub = climbSub;
    this.gearSub = gearSub;
    this.intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbSub, this.gearSub, this.intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSub.useClimbPistons(RobotContainer.xboxController.getRawButton(Constants.climbAxis));
    gearSub.useGearPistons(RobotContainer.xboxController.getRawButton(Constants.gearAxis));
    intakeSub.useIntakePistons(RobotContainer.xboxController.getRawButton(Constants.intakeAxis));
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
