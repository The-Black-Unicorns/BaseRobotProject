// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.objectVision.ObjectVision;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {

  public enum SuperStructureStates {
    TRAVEL
  }

  @AutoLogOutput(key = "SuperStructure/currentState")
  private SuperStructureStates currentState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/wantedState")
  private SuperStructureStates wantedState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/previousState")
  private SuperStructureStates previousState = SuperStructureStates.TRAVEL;

  public Command currentCommand = null;

  private final Drive drive;
  private final Leds leds;
  private final ObjectVision objectVision;
  private final Vision vision;

  private final Timer climbTimer = new Timer();

  public SuperStructure(Drive drive, Leds leds, ObjectVision ObjectVision, Vision vision) {
    this.drive = drive;
    this.leds = leds;
    this.objectVision = ObjectVision;
    this.vision = vision;

    SmartDashboard.putBoolean("Structure/got here", false);
    climbTimer.reset();
    climbTimer.stop();
  }

  @Override
  public void periodic() {
    previousState = currentState;
    if (wantedState != currentState) currentState = handleStateTransition(wantedState);
    wantedState = currentState;
    stateMachine();
  }

  private SuperStructureStates handleStateTransition(SuperStructureStates wantedState) {
    // TODO: add logic with algae in gripper

    return switch (wantedState) {
      case TRAVEL -> {
        yield SuperStructureStates.TRAVEL;
      }

      default -> {
        System.out.println("SuperStructure: Invalid state transition requested: " + wantedState);
        yield currentState;
      }
    };
  }

  private void stateMachine() {
    switch (currentState) {
      case TRAVEL -> travel();

      default -> {}
    }
  }

  private void travel() {
    if (currentCommand != null) currentCommand.cancel();
    drive.setState(DriveStates.FIELD_DRIVE);
    leds.setState(ledsStates.OFF);
  }

  private void setWantedState(SuperStructureStates wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(SuperStructureStates wantedState) {
    return new InstantCommand(() -> setWantedState(wantedState));
  }

  public Command setWantedStateCommand(SuperStructureStates wantedState, int scoreL) {
    return new InstantCommand(() -> setWantedState(wantedState));
  }
}
