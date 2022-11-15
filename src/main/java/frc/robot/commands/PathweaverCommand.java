package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class PathweaverCommand extends SequentialCommandGroup {
    Drivetrain m_drive;
    
    public PathweaverCommand(Drivetrain drive){
        m_drive=drive;

        addCommands();
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.drive.ksVolts,
            Constants.drive.kvVoltSecondsPerMeter,
            Constants.drive.kaVoltSecondsSquaredPerMeter),
            drive.getDifferentialDriveWheelSpeeds(),
            10);}
}
