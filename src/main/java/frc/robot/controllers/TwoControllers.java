package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwoControllers implements ControllerInterface {

  private final GenericHID controller;

  public TwoControllers() {
    controller = new GenericHID(0);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(3));
  }

  @Override
  public double xVelocityAnalog() {
    return controller.getRawAxis(3);
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getRawAxis(2);
  }

  @Override
  public double rotationVelocityAnalog() {
    return controller.getRawAxis(0);
  }
}
