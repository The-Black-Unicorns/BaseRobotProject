package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimulationController implements ControllerInterface {
  private final GenericHID controller;

  public SimulationController() {
    controller = new GenericHID(0);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(() -> controller.getRawButton(8) && controller.getPOV() == 0.0);
  }

  @Override
  public double xVelocityAnalog() {
    return controller.getRawAxis(0);
  }

  @Override
  public double yVelocityAnalog() {
    return -controller.getRawAxis(1);
  }

  @Override
  public double rotationVelocityAnalog() {
    return controller.getRawAxis(2);
  }
}
