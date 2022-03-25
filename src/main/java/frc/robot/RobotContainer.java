// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MotorCommand;
import frc.robot.commands.PistonCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController xboxController;
  public static Joystick stickLeft, stickRight;
  Compressor compressor;

  DriveSubsystem driveSub;
  IntakePistonSubsystem intakePistonSub;
  MotorSubsystem motorIntakeSub;
  TurretSubsystem turretSub;
  AutonomousSubsystem autoSub;

  DriveCommand driveCmd;
  PistonCommand pistonCmd;
  MotorCommand intakeCmd;
  AutonomousCommand autonomousCmd;

  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //subsystems
    driveSub = new DriveSubsystem();
    //intakePistonSub = new IntakePistonSubsystem();
    //motorIntakeSub = new MotorSubsystem();
    turretSub = new TurretSubsystem();
    autoSub = new AutonomousSubsystem(turretSub, driveSub);
    
    //command
    driveCmd = new DriveCommand(driveSub);
    //pistonCmd = new PistonCommand(intakePistonSub);
    //intakeCmd = new MotorCommand(motorIntakeSub, turretSub);

    autonomousCmd = new AutonomousCommand(autoSub, turretSub); // this needs autosub

    //controllers
    stickLeft = new Joystick(Constants.stickPortL);
    stickRight = new Joystick(Constants.stickPortR);
    xboxController = new XboxController(Constants.xboxPort);
  
    //defailt commands
    driveSub.setDefaultCommand(driveCmd);
    autoSub.setDefaultCommand(autonomousCmd);
    //intakePistonSub.setDefaultCommand(pistonCmd);
    //motorIntakeSub.setDefaultCommand(intakeCmd);

    //compressor = new Compressor(PneumaticsModuleType.REVPH);
    //compressor.disable();
    //compressor.enableDigital();
    
    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomousCmd;
  }
}
