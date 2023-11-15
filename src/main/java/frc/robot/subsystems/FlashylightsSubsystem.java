package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlashylightsSubsystem extends SubsystemBase {

    private Timer timer;

    private final byte stuff = 33;

    public FlashylightsSubsystem() {
        timer = new Timer();
        timer.start();
    }

    private final SerialPort arduino = new SerialPort(9600, SerialPort.Port.kUSB);

    @Override
    public void periodic()
    {
        if (timer.hasElapsed(5)) {
            timer.restart();

            arduino.write(new byte[]{stuff}, 1);
        }

        if (arduino.read(1)[0] == stuff) {
            System.out.println("hooray!");
        }
    }
}
