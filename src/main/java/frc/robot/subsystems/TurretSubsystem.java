package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class TurretSubsystem extends SubsystemBase {

    private CANSparkMax hood, launcher1, launcher2;
    private TalonSRX base;
    private DutyCycleEncoder hoodEncoder;
    private DutyCycleEncoder baseEncoder;

    private double baseZero;
    private double hoodZero;

    private PIDController basePID;
    private PIDController hoodPID;

    public TurretSubsystem() {
        hood = new CANSparkMax(hoodID, CANSparkMaxLowLevel.MotorType.kBrushless);
        base = new TalonSRX(turretBaseID);

        launcher1 = new CANSparkMax(turretLauncherID1, CANSparkMaxLowLevel.MotorType.kBrushless);
        launcher2 = new CANSparkMax(turretLauncherID2, CANSparkMaxLowLevel.MotorType.kBrushless);

        hoodEncoder = new DutyCycleEncoder(hoodEncoderID);
        baseEncoder = new DutyCycleEncoder(baseEncoderID);

        hoodPID = new PIDController(1, 0, 0);
        //basePID = new PIDController(1, 0, 0);

        //TODO or set to a constant that is the zero measurement
        baseZero = baseEncoder.getAbsolutePosition();
        hoodZero = hoodEncoder.getAbsolutePosition();

        launcher2.follow(launcher1);
    }

    @Override
    public void periodic() {
        //System.out.println(getYaw());
        System.out.println(hood.getEncoder().getPosition());
    }

    public void setYaw(double angle) {
        //TODO if this works test what it returns
        base.set(TalonSRXControlMode.Position , (angle / 360) + baseZero);
    }

    //public double getYaw() { return baseEncoder.getAbsolutePosition() - baseZero; }

    //TODO tune this pid as well
    public void setPitch(double angle) {
        //hood.set(hoodPID.calculate(hoodEncoder.getAbsolutePosition(), (angle / 360) + hoodZero));
        //TODO if this works test what it returns because if it works we dont need to mount separate encoders
        hood.getPIDController().setReference((angle / 360) + hoodZero, CANSparkMax.ControlType.kPosition);
    }

    //public double getPitch() { return hoodEncoder.getAbsolutePosition() - hoodZero; }

    public void setLauncherRPM(int rpm) {
        //TODO tune pid for this
        launcher1.getPIDController().setReference(rpm * launcherRatio, CANSparkMax.ControlType.kVelocity);
        //launcher2.getPIDController().setReference(-rpm * launcherRatio, CANSparkMax.ControlType.kVelocity);
    }

    public boolean isSpunUp(int rpm) {
        return Math.abs(launcher1.getEncoder().getVelocity() - rpm) < 200;
    }

}
