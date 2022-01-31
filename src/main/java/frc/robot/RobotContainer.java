// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GearPistonCommand;
import frc.robot.commands.IntakePistonCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GearboxPistonSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController xboxController;
  public static Joystick stickLeft, stickRight;
  DriveSubsystem driveSub;
  DriveCommand driveCmd;
  GearboxPistonSubsystem gearPistonSub;
  GearPistonCommand gearPistonCmd;
  IntakePistonSubsystem intakePistonSub;
  IntakePistonCommand intakePistonCmd;
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSub = new DriveSubsystem();
    gearPistonSub = new GearboxPistonSubsystem();
    intakePistonSub = new IntakePistonSubsystem();

    driveCmd = new DriveCommand(driveSub);
    gearPistonCmd = new GearPistonCommand(gearPistonSub);
    intakePistonCmd = new IntakePistonCommand(intakePistonSub);

    stickLeft = new Joystick(Constants.stickPortL);
    stickRight = new Joystick(Constants.stickPortR);
    
    xboxController = new XboxController(Constants.xboxPort);
  

    driveSub.setDefaultCommand(driveCmd);
    gearPistonSub.setDefaultCommand(gearPistonCmd);
    intakePistonSub.setDefaultCommand(intakePistonCmd);
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
    return null;
  }
}
