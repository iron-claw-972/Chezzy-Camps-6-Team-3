package frc.robot.controls;


import frc.robot.commands.PIDCommand;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.Axis;;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);


  public static void configureControls() {
    //driver.get(Button.A).whenPressed(new PIDCommand());
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
