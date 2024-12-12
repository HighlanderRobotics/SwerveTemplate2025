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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.swerve.OdometryThreadIO.OdometryThreadIOInputs;

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

  public static AprilTagFieldLayout fieldTags;

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
}
