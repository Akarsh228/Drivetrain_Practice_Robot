// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // drive motors: 0, 16, 2, 3 for inf recharge game bot
    // drive motors: 1, 2, 3, 4 for 2022 bot
    // public static final int MOTOR_L1_ID = 1;

    //Motors/Controller
    public static final int MOTOR_L1_ID = 15;
    public static final int MOTOR_L2_ID = 14;
    public static final int MOTOR_R1_ID = 10;
    public static final int MOTOR_R2_ID = 4;
    public static final int XBOX_DRIVE_CONTROLLER_PORT = 0;

    // Encoders
    public static final double cpr = 0124; //Counts Per Revolution
    public static final double whd = 6; //WHeel Radius
}