package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PIDCommand;
import frc.robot.commands.PathweaverCommand;
import frc.robot.subsystems.Drivetrain;


public class ShuffleboardManager {

  Drivetrain drive = new Drivetrain();
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  
  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  NetworkTableEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();
  NetworkTableEntry autoWait = m_autoTab.add("Auto Wait", 0.0).getEntry();
  
  
  public void setup() {

    SequentialCommandGroup autoPaths = new SequentialCommandGroup(
      new PathweaverCommand("1_All dots path", drive, true), 
      new PathweaverCommand("2_Reverse", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false),
      new PathweaverCommand("3_All the points forward", drive, false),
      new PathweaverCommand("4_All the points back", drive, false)
    );

    m_autoCommand.addOption("PIDCommand", new PIDCommand(drive));
    m_autoCommand.addOption("1_All dots path", new PathweaverCommand("1_All dots path", drive, true));
    m_autoCommand.addOption("2_Reverse", new PathweaverCommand("2_Reverse", drive, false));
    m_autoCommand.addOption("3_All the points forward", new PathweaverCommand("3_All the points forward", drive, false));
    m_autoCommand.addOption("4_All the points back", new PathweaverCommand("4_All the points back", drive, false));
    m_autoCommand.addOption("Full Path",autoPaths);



    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_autoCommand.getSelected();
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    chooserUpdate();

    
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
