// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MotorCommand;
import frc.robot.commands.PistonCommand;
import frc.robot.subsystems.ClimbPistonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GearPistonSubsystem;
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
  GearPistonSubsystem gearPistonSub;
  IntakePistonSubsystem intakePistonSub;
  ClimbPistonSubsystem climbPistonSub;
  MotorSubsystem motorIntakeSub;
  TurretSubsystem turretSub;

  DriveCommand driveCmd;
  PistonCommand pistonCmd;
  MotorCommand intakeCmd;
  AutonomousCommand autonomousCmd;



  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //subsystems
    driveSub = new DriveSubsystem();
    //gearPistonSub = new GearPistonSubsystem();
    //intakePistonSub = new IntakePistonSubsystem();
    //climbPistonSub = new ClimbPistonSubsystem();
    //motorIntakeSub = new MotorSubsystem();
    //turretSub = new TurretSubsystem();
    
    //command
    driveCmd = new DriveCommand(driveSub);
    //pistonCmd = new PistonCommand(climbPistonSub, gearPistonSub, intakePistonSub);
    //intakeCmd = new MotorCommand(motorIntakeSub, turretSub);
    autonomousCmd = new AutonomousCommand(driveSub);

    //controllers
    stickLeft = new Joystick(Constants.stickPortL);
    stickRight = new Joystick(Constants.stickPortR);
    xboxController = new XboxController(Constants.xboxPort);
  
    //defailt commands
    driveSub.setDefaultCommand(driveCmd);
    // gearPistonSub.setDefaultCommand(pistonCmd);
    // intakePistonSub.setDefaultCommand(pistonCmd);
    // climbPistonSub.setDefaultCommand(pistonCmd);
    // motorIntakeSub.setDefaultCommand(intakeCmd);

    //compressor = new Compressor(PneumaticsModuleType.REVPH);
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
