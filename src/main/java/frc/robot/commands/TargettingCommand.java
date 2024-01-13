package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.*;

public class TargettingCommand extends CommandBase {
    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;

    public TargettingCommand(TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(turretSubsystem, visionSubsystem);
    }

    @Override
    public void execute() {
//        if (xbox.getAButton()) {
//            turretSubsystem.setPitch(visionSubsystem.getHoodAngle());
//        }
    }
}
