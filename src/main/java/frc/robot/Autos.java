// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final AutoFactory factory;

  public Autos(SwerveSubsystem swerve) {
    this.swerve = swerve;
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            new AutoBindings(), // Leave empty, we can do bindings per-auto
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
            });
  }

  public AutoFactory getFactory() {
    return factory;
  }

  public Command getNoneAuto() {
    var routine = factory.newRoutine("None");
    routine.active().onTrue(Commands.print("Running empty auto."));
    return routine.cmd();
  }

  public Command getTestTriangle() {
    var routine = factory.newRoutine("Test Triangle");
    var traj = routine.trajectory("Triangle Test");
    routine.active().whileTrue(Commands.sequence(routine.resetOdometry(traj), traj.cmd()));
    return routine.cmd();
  }

  public Command getTestSprint() {
    var routine = factory.newRoutine("Test Sprint");
    var traj = routine.trajectory("Sprint Test");
    routine.active().whileTrue(Commands.sequence(routine.resetOdometry(traj), traj.cmd()));
    return routine.cmd();
  }

  public Command getC1Cycle() {
    var routine = factory.newRoutine("Cycle RHS Start to C1");
    var startToC1 = routine.trajectory("RHStoC1");
    var C1toP1 = routine.trajectory("C1toP1");
    var P1toC1 = routine.trajectory("P1toC1");

    routine
        .active()
        .whileTrue(Commands.sequence(routine.resetOdometry(startToC1), startToC1.cmd()));
    routine
        .observe(startToC1.done())
        .onTrue(
            Commands.sequence(
                // score
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> C1toP1.getInitialPose().orElse(Pose2d.kZero))),
                C1toP1.cmd()));
    routine
        .observe(C1toP1.done())
        .onTrue(
            Commands.sequence(
                // intake
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> P1toC1.getInitialPose().orElse(Pose2d.kZero))),
                P1toC1.cmd()));
    routine
        .observe(P1toC1.done())
        .onTrue(
            Commands.sequence(
                // score
                Commands.waitSeconds(0.5)
                    .raceWith(
                        swerve.poseLockDriveCommand(
                            () -> C1toP1.getInitialPose().orElse(Pose2d.kZero))),
                P1toC1.cmd()));

    return routine.cmd();
  }
}
