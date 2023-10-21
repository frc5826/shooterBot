package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-avis");

    private double[] pos = new double[6];

    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
        pos = limelight.getEntry("campose").getDoubleArray(new double[6]);
    }

    public double getHoodAngle() {
        return Math.atan2(getYVel(), getXVel());
    }

    public double getLaunchVel() {
        return Math.sqrt((Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2)));
    }

    public double getZDistance() {
        return Math.sqrt((Math.pow(pos[0], 2) + Math.pow(pos[2], 2)));
    }

    public double getTime() {
        return getZDistance() / getXVel();
    }

    public double getXVel() {
        return getZDistance() + 2;
    }

    public double getYVel() {
        double t = getTime();
        return 4.9 * t - (-pos[1] / t);
    }
}
