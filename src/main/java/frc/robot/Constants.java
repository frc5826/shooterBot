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

    //<editor-fold desc="Drive">

    public static final double driveBaseMaxVolts = 12; //TODO
    public static final double driveBaseMaxVelocity = 3; //TODO

    public static final double driveBaseWidth = .516;
    public static final double driveBaseLength = .568;

    public static int[] speedControllers = {4, 3, 2, 1, 8, 7, 6, 5};

    public static final double[] offsets = {166.3, 44.8, 321, 188}; //TODO

    public static final int frontLeftEncoder = 50;
    public static final int frontRightEncoder = 51;
    public static final int backLeftEncoder = 52;
    public static final int backRightEncoder = 53;

    //</editor-fold>

    //<editor-fold desc="Turret">

    public static final int shooterID = 0; //TODO

    public static final int hoodID = 9; //TODO
    public static final int turretBaseID = 10; //TODO

    public static final int turretLauncherID1 = 11; //TODO
    public static final int turretLauncherID2 = 12; //TODO

    public static final int hoodEncoderID = 54;
    public static final int baseEncoderID = 55;

    public static final double launcherRatio = 1; //TODO

    public static final double maxVelocity = 15; //TODO
    public static final int goalEndAngle = -60; //TODO

    //</editor-fold>

    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
