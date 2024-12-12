// Copyright 2021-2023 FRC 6328
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

package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;

import org.checkerframework.checker.units.qual.t;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.swerve.OdometryThreadIO.OdometryThreadIOInputs;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Samples;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalID;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionHelper;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.utils.Tracer;

public class SwerveSubsystem extends SubsystemBase {
  // Drivebase constants
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16);
  public static final double MAX_LINEAR_ACCELERATION = 8.0;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(21.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(21.25);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  public static final double MAX_ANGULAR_ACCELERATION = MAX_LINEAR_ACCELERATION / DRIVE_BASE_RADIUS;
  public static final double MAX_AUTOAIM_SPEED = MAX_LINEAR_SPEED / 4;
  // Hardware constants
  public static final int PIGEON_ID = 0;

  public static final double HEADING_VELOCITY_KP = 4.0;
  public static final double HEADING_VOLTAGE_KP = 4.0;
  
  public static final ModuleConstants frontLeft =
  new ModuleConstants(0, "Front Left", 0, 1, 0, Rotation2d.fromRotations(0.377930));
  public static final ModuleConstants frontRight =
  new ModuleConstants(1, "Front Right", 2, 3, 1, Rotation2d.fromRotations(-0.071289));
  public static final ModuleConstants backLeft =
  new ModuleConstants(2, "Back Left", 4, 5, 2, Rotation2d.fromRotations(0.550781));
  public static final ModuleConstants backRight =
  new ModuleConstants(3, "Back Right", 6, 7, 3, Rotation2d.fromRotations(-0.481689));
  
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules; // FL, FR, BL, BR
  private final OdometryThreadIO odoThread;
  private final OdometryThreadIOInputs odoThreadInputs = new OdometryThreadIOInputs();
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[0]);
  
  private final Vision[] cameras;
  public static AprilTagFieldLayout fieldTags;

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
  private SwerveDrivePoseEstimator estimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private double lastEstTimestamp = 0.0;
  private double lastOdometryUpdateTimestamp = 0.0;
  
  private Alert usingSyncOdometryAlert = new Alert("Using Sync Odometry", AlertType.kInfo);
  private Alert missingModuleData = new Alert("Missing Module Data", AlertType.kError);
  private Alert missingGyroData = new Alert("Missing Gyro Data", AlertType.kWarning);
  
  // Need this annotation so the alert doesn't get mad
  @SuppressWarnings("resource")
  public SwerveSubsystem(
      GyroIO gyroIO, VisionIO[] visionIOs, ModuleIO[] moduleIOs, OdometryThreadIO odoThread) {
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
    VisionIOSim.pose = () -> Pose3d.kZero;//this::getPose3d;

    // TODO: update to 2025 field
    try {
      fieldTags =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("vision" + File.separator + "2024-crescendo.json"));
      System.out.println("Successfully loaded tag map");
    } catch (Exception e) {
      System.err.println("Failed to load custom tag map");
      new Alert("Failed to load custom tag map", AlertType.kWarning).set(true);
      fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    }
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
      // Technically we could have not 4 modules worth of data here but if we have a design that goof we can deal later
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
   * Generates a set of samples without using the async thread.
   * Makes lots of Objects, so be careful when using it irl!
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
    var estPose = camera.update(camera.inputs.targets, camera.inputs.timestamp);
    if (estPose.isPresent()) {
        var visionPose = estPose.get().estimatedPose;
        // Sets the pose on the sim field
        camera.setSimPose(estPose, camera, isNewResult);
        Logger.recordOutput("Vision/Vision Pose From " + camera.getName(), visionPose);
        Logger.recordOutput("Vision/Vision Pose2d From " + camera.getName(), visionPose.toPose2d());
        estimator.addVisionMeasurement(
            visionPose.toPose2d(),
            camera.inputs.timestamp,
            VisionHelper.findVisionMeasurementStdDevs(estPose.get()));
        if (isNewResult) lastEstTimestamp = camera.inputs.timestamp;
      }
    }
  }
}
