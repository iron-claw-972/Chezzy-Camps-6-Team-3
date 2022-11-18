package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.time.LocalTime;
import frc.robot.subsystems.Drivetrain;


public class PIDVelocity {
    Drivetrain m_subsystem;
    Drivetrain d = new Drivetrain();
    public static double speed = 1200;
    public PIDVelocity(){
        PID();
    }

    public void PID(double targetVelocity)
    {
        double currentVelocity = m_leftMotor1.getVelocity();
    }
    
}
