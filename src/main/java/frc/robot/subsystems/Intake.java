package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    CANSparkMax motor = new CANSparkMax(69/420, MotorType.kBrushless);//int = id of motor(wild guess)
    public void setMotor(double power){
        motor.set(power);
    }
    

}