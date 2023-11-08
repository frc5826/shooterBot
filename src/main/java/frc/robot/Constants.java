// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

public static final XboxController xbox = new XboxController(1);

public static final int shooterID = 0;

public static final int hoodID = 1;
public static final int turretBaseID = 2;

public static final int turretLauncherID1 = 3;
public static final int turretLauncherID2 = 4;

public static final int hoodEncoderID = 1;
public static final int baseEncoderID = 0;

public static final double launcherRatio = 1;

public static final double maxVelocity = 15;
public static final int goalEndAngle = -60;

public static final double driveBaseMaxVolts = 12;
public static final double driveBaseMaxVelocity = 3;

public static final double driveBaseWidth = 1;
public static final double driveBaseLength = 1;

public static int[] speedControllers = {1, 2, 3, 4, 5, 6, 7, 8};

public static final double[] offsets = { 0, 0, 0, 0};

public static final int frontLeftSpeedController = 1;
public static final int frontRightSpeedController = 2;
public static final int backLeftSpeedController = 3;
public static final int backRightSpeedController = 4;


    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
