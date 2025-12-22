// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AllianceFlipping;
import frc.robot.SuperStructure.SuperStructureStates;
import frc.robot.autonomous.AutoTest;
import frc.robot.controllers.ControllerInterface;
import frc.robot.controllers.SimulationController;
import frc.robot.controllers.SingleXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.objectVision.ObjectVision;
import frc.robot.subsystems.objectVision.ObjectVisionIO;
import frc.robot.subsystems.objectVision.ObjectVisionIOPhoton;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Leds leds;
  private final ObjectVision ObjectVision;
  private final RobotState robotState;
  private SwerveDriveSimulation driveSimulation = null;

  private final SuperStructure structure;

  // Controller
  private final ControllerInterface controller;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    robotState = RobotState.getInstance();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        controller = new SingleXboxController();
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {},
                controller::xVelocityAnalog,
                controller::yVelocityAnalog,
                controller::rotationVelocityAnalog);
        leds = new Leds();
        ObjectVision = new ObjectVision(new ObjectVisionIOPhoton("herg", new Transform3d()));
        vision =
            new Vision(
                robotState::addVisionObservation,
                new VisionIO[] {
                  /*
                  new VisionIOPhoton("camera0", VisionConstants.robotToCamera0),
                  new VisionIOPhoton("camera1", VisionConstants.robotToCamera1),
                  new VisionIOLimelight("limelight-tsachi", RobotState.getInstance()::getYaw)*/
                });
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        controller = new SimulationController();
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                (robotPose) -> driveSimulation.getSimulatedDriveTrainPose(),
                controller::xVelocityAnalog,
                controller::yVelocityAnalog,
                controller::rotationVelocityAnalog);
        leds = new Leds();
        ObjectVision = new ObjectVision(new ObjectVisionIO() {});
        vision =
            new Vision(
                robotState::addVisionObservation,
                new VisionIO[] {
                  new VisionIOPhotonSim(
                      "camera0",
                      VisionConstants.robotToCamera0,
                      driveSimulation::getSimulatedDriveTrainPose),
                  // new VisionIOPhotonSim(
                  //     "camera1",
                  //     VisionConstants.robotToCamera1,
                  //     driveSimulation::getSimulatedDriveTrainPose)
                  // new VisionIOTest()
                });
        break;

      default:
        // Replayed robot, disable IO implementations
        controller = new SimulationController();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {},
                controller::xVelocityAnalog,
                controller::yVelocityAnalog,
                controller::rotationVelocityAnalog);
        leds = new Leds();
        ObjectVision = new ObjectVision(new ObjectVisionIO() {});
        vision = new Vision(robotState::addVisionObservation, new VisionIO[] {});
        break;
    }

    structure = new SuperStructure(drive, leds, ObjectVision, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> RobotState.getInstance().resetPose(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                RobotState.getInstance()
                    .resetPose(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            AllianceFlipping.apply(Rotation2d.fromDegrees(180))));

    controller.resetGyroButton().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return new AutoTest(drive, leds);
  }

  public void periodic() {
    SmartDashboard.putData("sub-drive", drive);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    // SmartDashboard.putData(
    //     "rgtrh",
    //     new AlignAlgaeReefCommand(drive, arm, elevator, gripper, false, () -> false, () ->
    // true));
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    RobotState.getInstance().resetPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public void autonomousInit() {
    structure.setWantedState(SuperStructureStates.AUTO);
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // Reset the simulation to the initial pose
    driveSimulation.setSimulationWorldPose(RobotState.getInstance().getEstimatedPose());
  }
}
