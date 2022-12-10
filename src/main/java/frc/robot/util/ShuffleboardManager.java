package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    chooserUpdate();

    m_autoTab.add("Auto Chooser", m_autoCommand);

    m_mainTab.add("Start Intake", new InstantCommand(() -> Robot.intake.setMotor(0.1)));
    m_mainTab.add("Stop Intake", new InstantCommand(() -> Robot.intake.setMotor(0.0)));
    
    // m_mainTab.addBoolean("Intake running: ", setIntakeStatus(intakeOn));
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  public void chooserUpdate() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
  }

  public BooleanSupplier setIntakeStatus(boolean intakeOn) {
    return () -> intakeOn;
  }

  public void setOuttakeStatus(boolean outtakeOn) {
    BooleanSupplier outtakeStatus = () -> outtakeOn;

    m_mainTab.addBoolean("Outtake running: ", outtakeStatus);
  }

  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }


}
