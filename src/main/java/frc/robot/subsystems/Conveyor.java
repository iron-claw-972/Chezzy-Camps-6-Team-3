package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;

public class Conveyor extends SubsystemBase {

    public boolean conveyorOn = false;

    CANSparkMax conveyor_motor = new CANSparkMax(Constants.ConveyorPort, MotorType.kBrushless);

    public void setMotor(double power) {

        if (power == 0) {
            conveyor_motor.set(0);
            conveyorOn = false;
        } else {
            conveyor_motor.set(power);
            conveyorOn = true;
        }

        conveyor_motor.set(power);
        
    }
}
