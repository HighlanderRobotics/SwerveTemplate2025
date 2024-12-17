// Copyright 2023-2025 FRC 8033
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

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.OdometryThreadIO.OdometryThreadIOInputs;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalID;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionHelper;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.utils.Tracer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveConstants constants;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules; // FL, FR, BL, BR
  private final OdometryThreadIO odoThread;
  private final OdometryThreadIOInputs odoThreadInputs = new OdometryThreadIOInputs();

  private SwerveDriveKinematics kinematics;

  private final Vision[] cameras;

  /** For delta tracking with PhoenixOdometryThread* */
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d rawGyroRotation = new Rotation2d();
  private Rotation2d lastGyroRotation = new Rotation2d();
  private SwerveDrivePoseEstimator estimator;
  private double lastEstTimestamp = 0.0;
  private double lastOdometryUpdateTimestamp = 0.0;

  private Alert usingSyncOdometryAlert = new Alert("Using Sync Odometry", AlertType.kInfo);
  private Alert missingModuleData = new Alert("Missing Module Data", AlertType.kError);
  private Alert missingGyroData = new Alert("Missing Gyro Data", AlertType.kWarning);

  public SwerveSubsystem(
      SwerveConstants constants,
      GyroIO gyroIO,
      VisionIO[] visionIOs,
      ModuleIO[] moduleIOs,
      OdometryThreadIO odoThread) {
    this.constants = constants;
    this.kinematics = new SwerveDriveKinematics(constants.getModuleTranslations());
    this.estimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    this.gyroIO = gyroIO;
    this.odoThread = odoThread;
    odoThread.start();
    cameras = new Vision[visionIOs.length];
    modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      cameras[i] = new Vision(visionIOs[i]);
    }

    // global static is mildly questionable
    VisionIOSim.pose = this::getPose3d;
  }

  public void periodic() {
    Tracer.trace(
        "SwervePeriodic",
        () -> {
          for (var camera : cameras) {
            Tracer.trace("Update cam inputs", camera::updateInputs);
            Tracer.trace("Process cam inputs", camera::processInputs);
          }
          Tracer.trace(
              "Update odo inputs",
              () -> odoThread.updateInputs(odoThreadInputs, lastOdometryUpdateTimestamp));
          Logger.processInputs("Async Odo", odoThreadInputs);
          if (!odoThreadInputs.sampledStates.isEmpty()) {
            lastOdometryUpdateTimestamp =
                odoThreadInputs
                    .sampledStates
                    .get(odoThreadInputs.sampledStates.size() - 1)
                    .timestamp();
          }
          Tracer.trace("update gyro inputs", () -> gyroIO.updateInputs(gyroInputs));
          for (int i = 0; i < modules.length; i++) {
            Tracer.trace(
                "SwerveModule update inputs from " + modules[i].getPrefix() + " Module",
                modules[i]::updateInputs);
          }
          Logger.processInputs("Swerve/Gyro", gyroInputs);
          for (int i = 0; i < modules.length; i++) {
            Tracer.trace(
                "SwerveModule periodic from " + modules[i].getPrefix() + " Module",
                modules[i]::periodic);
          }

          // Stop moving when disabled
          if (DriverStation.isDisabled()) {
            for (var module : modules) {
              module.stop();
            }
          }
          // Log empty setpoint states when disabled
          if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
          }

          Tracer.trace("Update odometry", this::updateOdometry);
          Tracer.trace("Update vision", this::updateVision);
        });
  }

  private void updateOdometry() {
    Logger.recordOutput("Swerve/Updates Since Last", odoThreadInputs.sampledStates.size());
    var sampleStates = odoThreadInputs.sampledStates;
    if (sampleStates.size() == 0 || sampleStates.get(0).values().isEmpty()) {
      usingSyncOdometryAlert.set(true);
      sampleStates = getSyncSamples();
    } else {
      usingSyncOdometryAlert.set(false);
    }

    for (var sample : sampleStates) {
      // Read wheel deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas =
          new SwerveModulePosition[4]; // change in positions since the last update
      boolean hasNullModulePosition = false;
      // Technically we could have not 4 modules worth of data here but if we have a design that
      // goof we can deal later
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        var dist = sample.values().get(new SignalID(SignalType.DRIVE, moduleIndex));
        if (dist == null) {
          // No value at this timestamp
          hasNullModulePosition = true;

          Logger.recordOutput("Odometry/Received Update From Module " + moduleIndex, false);
          break;
        }

        var rot = sample.values().get(new SignalID(SignalType.STEER, moduleIndex));
        if (rot == null) {
          // No value at this timestamp
          hasNullModulePosition = true;

          Logger.recordOutput("Odometry/Received Update From Module " + moduleIndex, false);
          break;
        }

        // all our data is good!
        modulePositions[moduleIndex] =
            new SwerveModulePosition(
                dist, Rotation2d.fromRotations(rot)); // gets positions from the thread, NOT inputs

        Logger.recordOutput("Odometry/Received Update From Module " + moduleIndex, true);
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // missing some data :(
      if (hasNullModulePosition) {
        missingModuleData.set(true);
        if (!gyroInputs.connected
            || sample.values().get(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID))
                == null) {
          missingGyroData.set(true);
          // no modules and no gyro so we're just sad :(
        } else {
          missingGyroData.set(false);
          // null here is checked by if clause
          rawGyroRotation =
              Rotation2d.fromDegrees(sample.values().get(new SignalID(SignalType.GYRO, -1)));
          lastGyroRotation = rawGyroRotation;
          Logger.recordOutput("Odometry/Gyro Rotation", lastGyroRotation);
          Tracer.trace(
              "update estimator",
              () ->
                  estimator.updateWithTime(
                      sample.timestamp(), rawGyroRotation, lastModulePositions));
        }
        continue;
      }

      // If we have all our module data . . .
      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      if (!gyroInputs.connected
          || sample.values().get(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID))
              == null) {
        missingGyroData.set(true);
        // We don't have a complete set of data, so just use the module rotations
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      } else {
        missingGyroData.set(false);
        rawGyroRotation =
            Rotation2d.fromDegrees(
                sample
                    .values()
                    .get(new SignalID(SignalType.GYRO, OdometryThreadIO.GYRO_MODULE_ID)));
        twist =
            new Twist2d(twist.dx, twist.dy, rawGyroRotation.minus(lastGyroRotation).getRadians());
      }
      // Apply the twist (change since last sample) to the current pose
      lastGyroRotation = rawGyroRotation;
      Logger.recordOutput("Odometry/Gyro Rotation", lastGyroRotation);
      // Apply update
      estimator.updateWithTime(sample.timestamp(), rawGyroRotation, modulePositions);
    }
  }

  /**
   * Generates a set of samples without using the async thread. Makes lots of Objects, so be careful
   * when using it irl!
   */
  private List<Samples> getSyncSamples() {
    return List.of(
        new Samples(
            Logger.getTimestamp(),
            Map.of(
                new SignalID(SignalType.DRIVE, 0), modules[0].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 0), modules[0].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 1), modules[1].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 1), modules[1].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 2), modules[2].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 2), modules[2].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 3), modules[3].getPosition().distanceMeters,
                new SignalID(SignalType.STEER, 3), modules[3].getPosition().angle.getRotations(),
                new SignalID(SignalType.GYRO, PhoenixOdometryThread.GYRO_MODULE_ID),
                    gyroInputs.yawPosition.getRotations())));
  }

  private void updateVision() {
    for (var camera : cameras) {
      boolean isNewResult = Math.abs(camera.inputs.timestamp - lastEstTimestamp) > 1e-5;
      var estPose =
          camera.update(
              camera.inputs.targets, camera.inputs.timestamp, constants.getFieldTagLayout());
      if (estPose.isPresent()) {
        var visionPose = estPose.get().estimatedPose;
        // Sets the pose on the sim field
        camera.setSimPose(estPose, camera, isNewResult);
        Logger.recordOutput("Vision/Vision Pose From " + camera.getName(), visionPose);
        Logger.recordOutput("Vision/Vision Pose2d From " + camera.getName(), visionPose.toPose2d());
        estimator.addVisionMeasurement(
            visionPose.toPose2d(),
            camera.inputs.timestamp,
            VisionHelper.findVisionMeasurementStdDevs(
                estPose.get(),
                constants.getVisionPointBlankStdDevs(),
                constants.getVisionDistanceFactor()));
        Pose3d[] tagPose3ds = new Pose3d[camera.inputs.targets.size()];
        for (int i = 0; i < camera.inputs.targets.size(); i++) {
          var target = camera.inputs.targets.get(i);
          tagPose3ds[i] = constants.getFieldTagLayout().getTagPose(target.getFiducialId()).get();
        }
        Logger.recordOutput("Vision/" + camera.getName() + " Target Pose3ds", tagPose3ds);
        if (isNewResult) lastEstTimestamp = camera.inputs.timestamp;
      }
    }
  }

  /** Returns the current pose estimator pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  /** Wraps {@link #getPose()} in a Pose3d. */
  public Pose3d getPose3d() {
    return new Pose3d(getPose());
  }

  /** Returns the current odometry rotation as returned by {@link #getPose()}. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states =
        Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    return states;
  }

  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
    speeds.toRobotRelativeSpeeds(getRotation());
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * Runs the drivetrain at the given robot relative speeds
   *
   * @param speeds robot relative target speeds
   * @param openLoop should the modules use open loop voltage to approximate the setpoint
   * @param moduleForcesX field relative force feedforward to apply to the modules. Must have the
   *     same number of elements as there are modules.
   * @param moduleForcesY field relative force feedforward to apply to the modules. Must have the
   *     same number of elements as there are modules.
   */
  private void drive(
      ChassisSpeeds speeds, boolean openLoop, double[] moduleForcesX, double[] moduleForcesY) {
    // Calculate module setpoints
    speeds.discretize(0.02);
    final SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, constants.getMaxLinearSpeed());

    Logger.recordOutput("Swerve/Target Speeds", speeds);
    Logger.recordOutput("Swerve/Speed Error", speeds.minus(getVelocity()));
    Logger.recordOutput(
        "Swerve/Target Chassis Speeds Field Relative",
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation()));

    // Send setpoints to modules
    final SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[modules.length];
    // Kind of abusing SwerveModuleState here but w/e
    final SwerveModuleState[] forceSetpoints = new SwerveModuleState[modules.length];
    for (int i = 0; i < optimizedSetpointStates.length; i++) {
      if (openLoop) {
        // Use open loop voltage control (teleop)
        // Heuristic to enable/disable FOC
        final boolean focEnable =
            Math.sqrt(
                    Math.pow(this.getVelocity().vxMetersPerSecond, 2)
                        + Math.pow(this.getVelocity().vyMetersPerSecond, 2))
                < constants.getMaxLinearSpeed()
                    * 0.9; // TODO tune the magic number (90% of free speed)
        optimizedSetpointStates[i] =
            modules[i].runVoltageSetpoint(
                new SwerveModuleState(
                    // Convert velocity to voltage with kv
                    setpointStates[i].speedMetersPerSecond * 12.0 / constants.getMaxLinearSpeed(),
                    setpointStates[i].angle),
                focEnable);

        // Have to put something here to log properly
        forceSetpoints[i] = new SwerveModuleState();
      } else {
        // Use closed loop current control (automated actions)
        // Calculate robot forces
        var robotRelForceX =
            moduleForcesX[i] * getRotation().getCos() - moduleForcesY[i] * getRotation().getSin();
        var robotRelForceY =
            moduleForcesX[i] * getRotation().getSin() + moduleForcesY[i] * getRotation().getCos();
        forceSetpoints[i] =
            new SwerveModuleState(
                Math.hypot(moduleForcesX[i], moduleForcesY[i]),
                new Rotation2d(moduleForcesX[i], moduleForcesY[i]));
        optimizedSetpointStates[i] =
            modules[i].runSetpoint(setpointStates[i], robotRelForceX, robotRelForceY);
      }
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/ForceSetpoints", forceSetpoints);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /**
   * Drive at a robot-relative speed.
   *
   * @param speeds the robot-relative speed reference.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get(), false, new double[4], new double[4]));
  }

  /**
   * Drive at a robot-relative speed.
   *
   * @param speeds the robot-relative speed reference.
   * @param xForces an array of forces (in Nm) along the field-relative x axis to add as a
   *     feedforward to the modules.
   * @param yForces an array of forces (in Nm) along the field-relative y axis to add as a
   *     feedforward to the modules.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocity(
      Supplier<ChassisSpeeds> speeds, Supplier<double[]> xForces, Supplier<double[]> yForces) {
    return this.run(() -> drive(speeds.get(), false, xForces.get(), yForces.get()));
  }

  /**
   * Drive at a field-relative speed.
   *
   * @param speeds the field-relative speed reference.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.driveVelocity(
        () -> {
          var speed = speeds.get();
          speed.toRobotRelativeSpeeds(getRotation());
          return speed;
        });
  }

  /**
   * Drive at a field-relative speed.
   *
   * @param speeds the field-relative speed reference.
   * @param xForces an array of forces (in Nm) along the field-relative x axis to add as a
   *     feedforward to the modules.
   * @param yForces an array of forces (in Nm) along the field-relative y axis to add as a
   *     feedforward to the modules.
   * @return a Command driving to the target speeds.
   */
  public Command driveVelocityFieldRelative(
      Supplier<ChassisSpeeds> speeds, Supplier<double[]> xForces, Supplier<double[]> yForces) {
    return this.driveVelocity(
        () -> {
          var speed = speeds.get();
          speed.toRobotRelativeSpeeds(getRotation());
          return speed;
        },
        xForces,
        yForces);
  }

  public Command driveTeleop(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          ChassisSpeeds speed = speeds.get();
          speed.toRobotRelativeSpeeds(getRotation());
          this.drive(speed, true, new double[4], new double[4]);
        });
  }
}
