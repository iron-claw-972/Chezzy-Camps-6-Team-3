package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class Intake {
    private CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
    private double startTime = Timer.getFPGATimestamp();

    public void takeBalls() {
        motor1.set(0.3);        // Sets motor power to +30% (forward)
        Timer.delay(5);         // Delays by 5 seconds
        motor1.set(0);          // Sets motor power to 0 (stops)
    }

}
