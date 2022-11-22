package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class PathweaverCommand extends SequentialCommandGroup {
    Drivetrain m_drive;
    
    public PathweaverCommand(Drivetrain drive){
        m_drive=drive;

        addCommands();
        TrajectoryConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
            Constants.drive.ksVolts,
            Constants.drive.kvVoltSecondsPerMeter,
            Constants.drive.kaVoltSecondsSquaredPerMeter),
            m_drive.getDifferentialDriveKinematics(),
            10);

            // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.auto.kMaxSpeedMetersPerSecond,
            Constants.auto.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_drive.getDifferentialDriveKinematics())
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("ExamplePath", new PathConstraints(4, 3));

           
            
    }
 
 public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    // in your code that will be used by all path following commands.
    
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              m_drive.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            m_drive.getPose(), // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(KS, KV, KA),
            m_drive.getDifferentialDriveKinematics(), // DifferentialDriveKinematics
            m_drive.getWheelSpeeds(), // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::outputVolts, // Voltage biconsumer
            eventMap, // This argument is optional if you don't use event markers
            this // Requires this drive subsystem
        )
    );
}

}
