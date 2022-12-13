package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Output extends SubsystemBase {
    CANSparkMax outtake_motor = new CANSparkMax(Constants.OuttakePort, MotorType.kBrushless);//int = id of motor(wild guess)

    public boolean outputOn = false;

    public void setMotor(double power) {
        if (power == 0) {
            outtake_motor.set(0);
            outputOn = false;
        } else {
            outtake_motor.set(power);
            outputOn = true;
        }
    }
    
    

}