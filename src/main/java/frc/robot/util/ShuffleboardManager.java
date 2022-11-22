package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.PIDCommand;
import frc.robot.subsystems.Drivetrain;


public class ShuffleboardManager {

  Drivetrain drive = new Drivetrain();
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  
  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();
  NetworkTableEntry autoWait = m_autoTab.add("Auto Wait", 0.0).getEntry();
  
  
  public void setup() {
    m_autoCommand.addOption("PIDCommand", new PIDCommand(drive));
    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_autoCommand.getSelected();
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    chooserUpdate();

    m_autoTab.add("Auto Chooser", m_autoCommand);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  public void chooserUpdate() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
  }

  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }

}
