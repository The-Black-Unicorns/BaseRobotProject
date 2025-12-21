package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleXboxController implements ControllerInterface {

  private final XboxController controller;

  public SingleXboxController() {
    controller = new XboxController(0);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(8) && controller.getPOV() == 0.0);
  }

  @Override
  public double xVelocityAnalog() {
    return controller.getLeftX();
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getLeftY();
  }

  @Override
  public double rotationVelocityAnalog() {
    return controller.getRightX();
  }
}
