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

        double y;
        double x;
        double rot;

        if (Math.abs(xbox.getLeftY()) > controllerDeadzone) {
            y = xbox.getLeftY() * driveBaseMaxVelocity;
        } else { y = 0; }

        if (Math.abs(xbox.getLeftX()) > controllerDeadzone) {
            x = xbox.getLeftX() * driveBaseMaxVelocity;
        } else { x = 0; }

        if (Math.abs(xbox.getRightX()) > controllerDeadzone) {
            rot = xbox.getRightX() * driveSubsystem.maxAngularVelocity;
        } else { rot = 0; }

        driveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-y, -x, -rot, driveSubsystem.getRotation()));

        if (xbox.getAButtonPressed()) {
            driveSubsystem.zeroGyroscope();
        }

    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
