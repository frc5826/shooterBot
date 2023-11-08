package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.*;

public class DriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    public DriveCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double y = xbox.getLeftY() * driveBaseMaxVelocity;
        double x = xbox.getLeftX() * driveBaseMaxVelocity;
        double rot = xbox.getRightX() * driveSubsystem.maxAngularVelocity;

        driveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, driveSubsystem.getRotation()));

    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
