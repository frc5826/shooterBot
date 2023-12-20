package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {

    public final double maxAngularVelocity = driveBaseMaxVelocity /
            Math.hypot(driveBaseWidth / 2, driveBaseLength / 2);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(driveBaseWidth / 2.0, driveBaseLength / 2.0),
            new Translation2d(driveBaseWidth / 2.0, -driveBaseLength / 2.0),
            new Translation2d(-driveBaseWidth / 2.0, driveBaseLength / 2.0),
            new Translation2d(-driveBaseWidth / 2.0, -driveBaseLength / 2.0)
    );

    private SwerveDriveOdometry odometry = null;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DriveSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        MkModuleConfiguration configuration = MkModuleConfiguration.getDefaultSteerNEO();
        configuration.setDriveCurrentLimit(30);

        frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 2)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, speedControllers[0])
                .withSteerMotor(MotorType.NEO, speedControllers[1])
                .withSteerEncoderPort(frontLeftEncoder)
                .withSteerOffset(offsets[0])
                .build();

        frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 2)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, speedControllers[2])
                .withSteerMotor(MotorType.NEO, speedControllers[3])
                .withSteerEncoderPort(frontRightEncoder)
                .withSteerOffset(offsets[1])
                .build();

        backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 2)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, speedControllers[4])
                .withSteerMotor(MotorType.NEO, speedControllers[5])
                .withSteerEncoderPort(backLeftEncoder)
                .withSteerOffset(offsets[2])
                .build();

        backRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 2)
                        .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, speedControllers[6])
                .withSteerMotor(MotorType.NEO, speedControllers[7])
                .withSteerEncoderPort(backRightEncoder)
                .withSteerOffset(offsets[3])
                .build();

        setRampRates(frontLeftModule, frontRightModule, backLeftModule, backRightModule);

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyro.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    }

    private void setRampRates(SwerveModule... swerveModules) {
        for(var s : swerveModules) {
            ((CANSparkMax) s.getDriveMotor()).setOpenLoopRampRate(Constants.rampRate);
        }
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
                Rotation2d.fromDegrees(gyro.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
        );
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        odometry.update(
                Rotation2d.fromDegrees(gyro.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / driveBaseMaxVelocity * driveBaseMaxVolts, states[0].angle.getRadians()/* - Math.toRadians(30) */);
        frontRightModule.set(states[1].speedMetersPerSecond / driveBaseMaxVelocity * driveBaseMaxVolts, states[1].angle.getRadians()/* - Math.toRadians(175) */);
        backLeftModule.set(states[2].speedMetersPerSecond / driveBaseMaxVelocity * driveBaseMaxVolts, states[2].angle.getRadians()/* + Math.toRadians(91.3) */);
        backRightModule.set(states[3].speedMetersPerSecond / driveBaseMaxVelocity * driveBaseMaxVolts, states[3].angle.getRadians()/* + Math.toRadians(169) */);
    }


}
