// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //chassi motor ports
    public static final int vPortR1 = 0;
    public static final int vPortR2 = 2;
    public static final int vPortR3 = 1;

    public static final int vPortL1 = 3;
    public static final int vPortL2 = 5;
    public static final int vPortL3 = 4;

    //joystick ports
    public static final int stickPortL = 1;
    public static final int stickPortR = 2;
    
    //xbox
    public static final int xboxPort = 0;

    //change gear pistons
    public static final int solenoidPort1 = 0;
    public static final int solenoidPort2 = 1;

    //lower intake pistons
    public static final int intakeSolonoidPort1 = 4;
    public static final int intakeSolonoidPort2 = 5;

    //climb pistons
    public static final int climbSolenoidPort1 = 6;
    public static final int climbSolenoidPort2 = 7;

    public static final int intakeMotorPort = 6;
    public static final int beltMotorPort = 7;
    public static final int pistonMotorPort1 = 8;
    public static final int pistonMotorPort2 = 9;

    public static final int shootPort1 = 8;
    public static final int shootPort2 = 9;
    public static final int turretRotatePort = 7;

    public static final int sonicDIO1 = 1;
    public static final int sonicDIO2 = 2;
}
