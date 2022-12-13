package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    Drivetrain m_drive;
    double m_distance;
    PIDController m_rightPID;
    PIDController m_leftPID;
    double m_encoderLeft;
    double m_encoderRight;

    public DriveDistance(Drivetrain drivetrain, double Distance) {
        m_drive = drivetrain;
        m_distance = m_distance;
        m_rightPID = new PIDController(Constants.drive.kPPosition, Constants.drive.kIPosition, Constants.drive.kDPosition);
        m_leftPID = new PIDController(Constants.drive.kPPosition, Constants.drive.kIPosition, Constants.drive.kDPosition);
        m_leftPID.setTolerance(5, 10);
        m_rightPID.setTolerance(5, 10);
    }

    public void initialize() {
        m_encoderLeft = m_drive.leftEncodersPosition();
        m_encoderRight = m_drive.rightEndcodersPosition();
    }

    public void execute() {
        m_drive.tankDrive(m_leftPID.calculate(m_drive.leftEncodersPosition(), m_encoderLeft+m_distance), m_rightPID.calculate(m_drive.rightEndcodersPosition(), m_encoderRight+m_distance));
    }

    public void end(Boolean isInterrupted) {
        m_drive.tankDrive(0,0);
    }

    public boolean isFinished() {
        return m_leftPID.atSetpoint() && m_rightPID.atSetpoint();
    }

}
