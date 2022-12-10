package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class Intake extends SubsystemBase{
    
    public boolean intakeOn = false; 

    CANSparkMax intake_motor = new CANSparkMax(Constants.IntakePort, MotorType.kBrushless);//int = id of motor(wild guess)
    public void setMotor(double power){
        
        if (power == 0) {
            intake_motor.set(0);
            intakeOn = false;
        } else {
            intake_motor.set(power);
            intakeOn = true;
        }
    }
    

}