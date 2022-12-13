package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.Axis;

// SlewRateLimiter filter=new SlewRateLimiter(0.5);

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {
    // driver.get(Button.A).whenPressed(new DoNothing());
    driver.get(Button.A).whenPressed(new InstantCommand(() -> Robot.intake.setMotor(0.1)));
    driver.get(Button.A).whenReleased(new InstantCommand(() -> Robot.intake.setMotor(0)));
    driver.get(Button.B).whenPressed(new InstantCommand(() -> Robot.intake.setMotor(-0.1)));
    driver.get(Button.B).whenReleased(new InstantCommand(() -> Robot.intake.setMotor(0)));
    driver.get(Button.X).whenPressed(new InstantCommand(() -> Robot.output.setMotor(0.1)));
    driver.get(Button.X).whenReleased(new InstantCommand(() -> Robot.output.setMotor(0)));
    driver.get(Button.Y).whenPressed(new InstantCommand(() -> Robot.output.setMotor(-0.1)));
    driver.get(Button.Y).whenReleased(new InstantCommand(() -> Robot.output.setMotor(0)));

  }

  public static double getLeftPower() {
    return driver.get(Axis.LEFT_Y);
  }

  public static double getRightPower() {
    return driver.get(Axis.RIGHT_Y);
  }
}
