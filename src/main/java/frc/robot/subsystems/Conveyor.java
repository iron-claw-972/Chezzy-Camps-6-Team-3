package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;

public class Conveyor extends SubsystemBase {
    CANSparkMax conveyor_motor = new CANSparkMax(Constants.ConveyorPort, MotorType.kBrushless);
    public void setMotor(double power) {
        conveyor_motor.set(power);
        
    }
}
