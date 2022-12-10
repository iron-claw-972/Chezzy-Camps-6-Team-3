package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.util.ShuffleboardManager;
import frc.robot.Robot;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  public static void configureControls() {
    operator.get(Button.A).whenPressed(new InstantCommand(() -> {
      Robot.conveyor.setMotor(0.1);
      // Robot.shuffleboard.setIntakeStatus(true);
    
    }));
    Robot.shuffleboard.setIntakeStatus(true);
    operator.get(Button.A).whenReleased(new InstantCommand(() -> {
      Robot.conveyor.setMotor(0);
      // Robot.shuffleboard.setIntakeStatus(false);
    }));
  }

}
