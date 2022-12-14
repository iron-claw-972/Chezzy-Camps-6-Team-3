package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArcadeDrive1 extends CommandBase {
  private final Drivetrain m_subsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive1(Drivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.drive.arcadeDrive(Robot.dr.getRawThrottleValue(), Robot.dr.getRawTurnValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     Robot.drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Robot.dr.ButtonA();
  }
  
}
