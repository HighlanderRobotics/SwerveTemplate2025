// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
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
              Logger.recordOutput("Choreo/Active Traj", traj.getPoses());
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
}
