/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  private final WPI_TalonFX m_rightMotor1;
  private final WPI_TalonFX m_leftMotor2;
  private final WPI_TalonFX m_rightMotor2;
  

 

  public Drivetrain() {
    m_leftMotor1 = MotorFactory.createTalonFX(Constants.drive.kLeftMotor1);
    m_rightMotor1 = MotorFactory.createTalonFX(Constants.drive.kRightMotor1);
    m_leftMotor2 = MotorFactory.createTalonFX(Constants.drive.kLeftMotor2);
    m_rightMotor2 = MotorFactory.createTalonFX(Constants.drive.kRightMotor2);

    m_rightMotor1.setInverted(true);


  }

  /**
   * Drives the robot using tank drive controls
   * Tank drive is slightly easier to code but less intuitive to control, so this is here as an example for now
   * @param leftPower the commanded power to the left motors
   * @param rightPower the commanded power to the right motors
   */
  public void tankDrive(double leftPower, double rightPower) {
    m_leftMotor1.set(ControlMode.PercentOutput, leftPower);
    m_rightMotor1.set(ControlMode.PercentOutput, rightPower);
  }

  public double averageLeftPosition()
  {
      return(m_leftMotor1.getSelectedSensorPosition();
  }

  public double averageRightPosition()
  {
      return(m_rightMotor1.getSelectedSensorPosition();
  }
  public double averagePosition()
  {
      return(m_leftMotor1.getSelectedSensorPosition() + m_rightMotor1.getSelectedSensorPosition());
  }

  

  

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward the commanded forward movement
   * @param turn the commanded turn rotation
   */
  public void arcadeDrive(double throttle, double turn) {
    m_leftMotor1.set((throttle - turn)*0.5);
    m_rightMotor1.set((throttle - turn)*0.5);
  }
}
