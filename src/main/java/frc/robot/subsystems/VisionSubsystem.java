package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.goalEndAngle;
import static frc.robot.Constants.maxVelocity;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-avis");

    private double[] pos = new double[6];

    private double vel;
    private double hoodAngle;

    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
        pos = limelight.getEntry("campose").getDoubleArray(new double[6]);
    }

    public double getVelocity() {
        getReasonableLaunchVelocity();
        return vel;
    }

    public double getHoodAngle() {
        getReasonableLaunchVelocity();
        return hoodAngle;
    }

    public double getUnreasonableLaunchVelocity() { return 3 * Math.pow(10, 8); }

    public void getReasonableLaunchVelocity() {
        int endAngle = goalEndAngle;
        double vel = calculateLaunchVelocity(endAngle);

        while (vel > maxVelocity) {
            endAngle -= 1;
            vel = calculateLaunchVelocity(endAngle);
        }

        this.hoodAngle = getHoodAngleTwoPointOh(endAngle);
        this.vel = vel;
    }

    public double calculateLaunchVelocity(double S) {
        double d = pos[2];
        double h = pos[0];
        double a = getHoodAngleTwoPointOh(S);

        return Math.sqrt(-(9.8 * Math.pow(d, 2) * (1 + Math.pow(Math.tan(a) ,2))) / 2 * h - 2 * d * Math.tan(a));
    }

    public double getHoodAngleTwoPointOh(double S) {
        double d = pos[2];
        double h = pos[0];

        return Math.atan2((Math.tan(S) * d - (2 * h)), -d);
    }

//    public double getHoodAngle() {
//        return Math.atan2(getYVel(), getXVel());
//    }
//
//    public double getLaunchVel() {
//        return Math.sqrt((Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2)));
//    }
//
//    public double getZDistance() {
//        return Math.sqrt((Math.pow(pos[0], 2) + Math.pow(pos[2], 2)));
//    }
//
//    public double getTime() {
//        return getZDistance() / getXVel();
//    }
//
//    public double getXVel() {
//        return (getZDistance() * 0.75);
//    }
//
//    public double getYVel() {
//        double t = getTime();
//        return 4.9 * t - (-pos[1] / t);
//    }

}
