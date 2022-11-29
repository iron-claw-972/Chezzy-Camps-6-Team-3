package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import javax.swing.text.AbstractDocument.LeafElement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDCommand extends CommandBase {

  // TODO 4.2: replace ExampleSubsystem with your created ExtraSubsystem, or with Drivetrain. change the name to something better
  Drivetrain m_subsystem;
  Drivetrain d = new Drivetrain();
  

  // TODO 4.2: Add a parameter that asks for the setpoint
  
  public PIDCommand(Drivetrain subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    // TODO 4.2: replace above ExampleSubsystem with your created ExtraSubsystem, or with Drivetrain.
  }
  public void initialize() {
    // TODO 4.2: zero encoders and rest the PID controller before starting
      d.zero();
      d.reset();
  }

  public void execute() {
    // TODO 4.2: Make the PID control loop
    d.periodic();

  }

  public void end(boolean interrupted) {
    // TODO 4.2: when the command ends, the motors should stop spinning
     d.stop();
  }

  public boolean isFinished() {
    // TODO 4.2: check if the PID is finished though the PID controller
    if(d.getPidone() == true)
    {
      return true;
    }
    return false;
  }
}
