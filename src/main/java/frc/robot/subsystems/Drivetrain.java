/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;


public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  private final WPI_TalonFX m_rightMotor1;
  private final WPI_TalonFX m_leftMotor2;
  private final WPI_TalonFX m_rightMotor2;

  

  PIDController m_pid = new PIDController(Constants.drive.kP,Constants.drive.kI, Constants.drive.kD);
  PIDController m_Vpid = new PIDController(Constants.drive.kP,Constants.drive.kI, Constants.drive.kD);
  boolean pidOn = false;
  boolean vpidOn = false;
  double sp = 1000;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26.0));
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
  DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
  // the GYRO
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  DifferentialDriveOdometry m_odometry;
  //Starting position
  Pose2d m_pose2d = new Pose2d(1,1,ahrs.getRotation2d());
  double leftVelocity = wheelSpeeds.leftMetersPerSecond;
  double rightVelocity = wheelSpeeds.rightMetersPerSecond;
  PIDController m_pidLeft = new PIDController(0, 0, 0);
  PIDController m_pidRight = new PIDController(0, 0, 0);

  public Drivetrain() {
    m_pid.setTolerance(5, 10);
    m_leftMotor1 = MotorFactory.createTalonFX(Constants.drive.kLeftMotor1);
    m_rightMotor1 = MotorFactory.createTalonFX(Constants.drive.kRightMotor1);
    m_leftMotor2 = MotorFactory.createTalonFX(Constants.drive.kLeftMotor2);
    m_rightMotor2 = MotorFactory.createTalonFX(Constants.drive.kRightMotor2);


    m_rightMotor1.setInverted(true);



    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);
    m_rightMotor1.configSelectedFeedbackCoefficient(Constants.drive.ConversionDistanceMeters);
    m_odometry=new DifferentialDriveOdometry(ahrs.getRotation2d());

  }

  /**
   * Drives the robot using tank drive controls
   * Tank drive is slightly easier to code but less intuitive to control, so this is here as an example for now
   * @param leftPower the commanded power to the left motors
   * @param rightPower the commanded power to the right motors
   */
 
  public void Vpid(double targetVelocity)
  {
    if(vpidOn == true)
    {
        m_leftMotor1.set(m_Vpid.calculate(m_leftMotor1.getSelectedSensorVelocity(), targetVelocity ));
        m_rightMotor1.set(m_Vpid.calculate(m_rightMotor1.getSelectedSensorVelocity(), targetVelocity));
    }
    else
    {
        m_leftMotor1.stopMotor();
        m_rightMotor1.stopMotor();
    }
  }
  @Override
   public void periodic()
   {
    if(pidOn == true)
    {
        m_leftMotor1.set(m_pid.calculate(m_leftMotor1.getSelectedSensorPosition(), sp));
        m_rightMotor1.set(m_pid.calculate(m_rightMotor1.getSelectedSensorPosition(), sp));
    }
    else
    {
        m_leftMotor1.stopMotor();
        m_rightMotor1.stopMotor();
    }
    
    m_odometry.update(ahrs.getRotation2d(), getleftEncoderdistanceMeters(), getRightEncoderdistanceMeters());

  }
   public void tankDrive(double leftPower, double rightPower) {
    m_leftMotor1.set(ControlMode.PercentOutput, leftPower);
    m_rightMotor1.set(ControlMode.PercentOutput, rightPower);
  }

  public double averageLeftPosition()
  {
      return(m_leftMotor1.getSelectedSensorPosition());
  }

  public double averageRightPosition()
  {
      return(m_rightMotor1.getSelectedSensorPosition());
  }
  public double averagePosition()
  {
      return(m_leftMotor1.getSelectedSensorPosition() + m_rightMotor1.getSelectedSensorPosition());
  }
  public void stopMotors()
  {
    m_leftMotor1.set(0);
    m_rightMotor1.set(0);
  }
  
  

  

  

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward the commanded forward movement
   * @param turn the commanded turn rotation
   */
  
  public void arcadeDrive(double throttle, double turn) {
    m_leftMotor1.set((throttle-turn*0.5));
    m_rightMotor1.set((throttle+turn)*0.5);
  }
  
  public void zero()
   {
      m_leftMotor1.setSelectedSensorPosition(0.0);
      m_rightMotor1.setSelectedSensorPosition(0.0);
   } 
   public double getEncoder()
   {
      
      return (m_leftMotor1.getSelectedSensorPosition() + m_rightMotor1.getSelectedSensorPosition())/2;
   }
   public void forward()
   {
     arcadeDrive(0.3, 0);
   }
   public void backward()
   {
     arcadeDrive(-0.3, 0);
   }
   public void stop()
   {
      m_leftMotor1.stopMotor();
      m_rightMotor1.stopMotor();
   }

  // TODO 4.1: write three functions, one for setting the setpoint, and one for setting whether the pid is enabled. The last one is a function to reset the PID with pid.reset()
  public void setpoint(double setpoint)
  {
      setpoint = sp; 
  }
  public void enable(boolean enable)
  {
      pidOn = enable; 
  }
  public void reset()
  {
    m_pid.reset();
  }
  public boolean getPidone(){
    return m_pid.atSetpoint();
  }
  

  public double getleftEncoderdistanceMeters(){

    double distanceinches=m_leftMotor1.getSelectedSensorPosition()*2048*12/62*4*Math.PI;
    return Units.inchesToMeters(distanceinches);


  }
  public double getRightEncoderdistanceMeters(){

    double distanceinches=m_rightMotor1.getSelectedSensorPosition()*2048*12/62*4*Math.PI;
    return Units.inchesToMeters(distanceinches);


  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetEncoders() {
    m_rightMotor1.setSelectedSensorPosition(0.0);
    m_leftMotor1.setSelectedSensorPosition(0.0);

  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, ahrs.getRotation2d());
  }
  public double getAverageEncoderDistance() {
    return ( getleftEncoderdistanceMeters()+ getRightEncoderdistanceMeters()) / 2.0;
  }
  public double getleftencodervalue(){
    return m_leftMotor1.getSelectedSensorPosition();
  }
  public double getrightencodervalue(){
    return m_rightMotor1.getSelectedSensorPosition();
  }
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_rightMotor1.getSelectedSensorVelocity(), m_leftMotor1.getSelectedSensorVelocity());
  }
  public DifferentialDriveWheelSpeeds GetDifferentialDriveWheelSpeeds() {
    return wheelSpeeds;
  }
  public Pose2d getpose2d() {
    return m_pose2d;
  }
  public DifferentialDriveKinematics getDifferentialDriveKinematics(){

    return kinematics;
  }


 

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor1.setVoltage(leftVolts);
    m_rightMotor1.setVoltage(rightVolts);
    //m_drive.feed();?
  }

}

