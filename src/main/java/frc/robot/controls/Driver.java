package frc.robot.controls;


import frc.robot.commands.PIDCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.Axis;;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);


  public static void configureControls() {
    driver.get(Button.A).whenPressed(new InstantCommand(() -> Robot.intake.setMotor(0.1)));
    driver.get(Button.A).whenReleased(new InstantCommand(() -> Robot.intake.setMotor(0)));
    driver.get(Button.B).whenPressed(new InstantCommand(() -> Robot.intake.setMotor(-0.1)));
    driver.get(Button.B).whenReleased(new InstantCommand(() -> Robot.intake.setMotor(0)));
    
    driver.get(Button.X).whenPressed(new InstantCommand(() -> Robot.output.setMotor(0.1)));
    driver.get(Button.X).whenReleased(new InstantCommand(() -> Robot.output.setMotor(0)));
    driver.get(Button.Y).whenPressed(new InstantCommand(() -> Robot.output.setMotor(-0.1)));
    driver.get(Button.Y).whenReleased(new InstantCommand(() -> Robot.output.setMotor(0)));

    driver.get(Button.LB).whenPressed(new InstantCommand(() -> Robot.conveyor.setMotor(0.1)));
    driver.get(Button.LB).whenReleased(new InstantCommand(() -> Robot.conveyor.setMotor(0.0)));
  }
  public double getRawThrottleValue() {
    return driver.get(Axis.LEFT_Y);
  }

  public double getRawTurnValue() {
    return driver.get(Axis.RIGHT_X);
  }

  public static double getRawLeft() {
    return driver.get(Axis.LEFT_Y);
  }

  public static double getRawRight() {
    return driver.get(Axis.RIGHT_Y);
  }
  public boolean ButtonA()
  {
    boolean a = false;
    driver.get(Button.A).whenPressed(null, a = true);
    return a;
  }
}
