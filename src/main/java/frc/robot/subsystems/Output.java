package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Output extends SubsystemBase{
    CANSparkMax motor = new CANSparkMax(Constants.OuttakePort, MotorType.kBrushless);//int = id of motor(wild guess)
    public void setMotor(double power){
        motor.set(power);
    }
    
    

}