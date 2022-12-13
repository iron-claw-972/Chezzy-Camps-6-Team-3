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
    //operator.get(Button.A).whenPressed(new DoNothing());
    operator.get(Button.A).whenPressed(new InstantCommand(() -> Robot.intake.setMotor(0.1)));
    operator.get(Button.A).whenReleased(new InstantCommand(() -> Robot.intake.setMotor(0)));
    operator.get(Button.B).whenPressed(new InstantCommand(() -> Robot.intake.setMotor(-0.1)));
    operator.get(Button.B).whenReleased(new InstantCommand(() -> Robot.intake.setMotor(0)));
    
    operator.get(Button.X).whenPressed(new InstantCommand(() -> Robot.output.setMotor(0.1)));
    operator.get(Button.X).whenReleased(new InstantCommand(() -> Robot.output.setMotor(0)));
    operator.get(Button.Y).whenPressed(new InstantCommand(() -> Robot.output.setMotor(-0.1)));
    operator.get(Button.Y).whenReleased(new InstantCommand(() -> Robot.output.setMotor(0)));

    operator.get(Button.LB).whenPressed(new InstantCommand(() -> Robot.conveyor.setMotor(0.1)));
    operator.get(Button.LB).whenReleased(new InstantCommand(() -> Robot.conveyor.setMotor(0.0)));
  }

}
