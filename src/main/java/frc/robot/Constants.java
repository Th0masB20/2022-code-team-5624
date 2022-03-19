// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    //2 falcon - turret shoot
    public static final int CANPortR1 = 3;
    public static final int CANPortR2 = 5;

    public static final int CANPortL1 = 0;
    public static final int CANPortL2 = 1;

    //joystick ports
    public static final int stickPortL = 1;
    public static final int stickPortR = 2;
    
    //xbox
    public static final int xboxPort = 0;

    //lower intake pistons
    public static final int intakeSolonoidPort1 = 2;
    public static final int intakeSolonoidPort2 = 3;

    //turret motors
    //all can id
    public static final int shootPort1 = 0;
    public static final int shootPort2 = 9;
    public static final int turretRotatePort = 7;

    //intake
    public static final int intakeMotorPort = 6;
    public static final int beltMotorPort = 12;

    public static final int pistonMotorPort1 = 8;
    public static final int pistonMotorPort2 = 9;

    /*
    public static final int sonicDIO1 = 1;
    public static final int sonicDIO2 = 2;
    */
}
