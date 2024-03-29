package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    public double maximumSpeed = 1.5;

    public double maximumAngularVel = 3;

    public SwerveSubsystem(File directory) {

        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1);

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);
    }

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
    {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians(), maximumSpeed);
    }

    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
    }

    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    public void postTrajectory(Trajectory trajectory)
    {
        swerveDrive.postTrajectory(trajectory);
    }

    public Rotation2d getHeading()
    {
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getFieldVelocity()
    {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    public void lock()
    {
        swerveDrive.lockPose();
    }

}
