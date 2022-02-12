// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends CommandBase {

  DriveSubsystem driveSub;
  MotorSubsystem motorSub;
  private double startTime;

  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(DriveSubsystem drive, MotorSubsystem motors) {
    driveSub = drive;
    motorSub = motors;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motorSub);
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp();

    //AFTER FIRST SHOT
    
    // EXAMPLE SEEKING PROGRAM
    // float tv = table->GetNumber("tv");
    // float tx = table->GetNumber("tx");

    // float steering_adjust = 0.0f;
    // if (tv == 0.0f)
    // {
    //         // We don't see the target, seek for the target by spinning in place at a safe speed.
    //         steering_adjust = 0.3f;
    // }
    // else
    // {
    //         // We do see the target, execute aiming code
    //         float heading_error = tx;
    //         steering_adjust = Kp * tx;
    // }

    // driveSub.drive(steering_adjust, steering_adjust);
    // [push movement code + time in stack]

    // DISTANCE  (maybe use moving average for ir signal)
    // if(ir_signal < calibrated_distance){
    //   driveSub.drive(0.0, 0.0);
    // }else{
    //   [Add timing]
    //   motorSub.useIntakeMotor(1.0);
    // }
    
    // Reverse movement code +time stack

    // SHOOT
    // motorSub.useBeltMotor(1.0);
     


    if(time - startTime < 3){
        driveSub.drive(0.5, 0.5);
    }else{
        driveSub.drive(0.0, 0.0);
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
