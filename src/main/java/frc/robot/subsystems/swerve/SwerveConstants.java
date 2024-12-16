package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.vision.Vision.VisionConstants;

public interface SwerveConstants {
  // Drivebase Constants
  public double getMaxLinearSpeed();

  public double getMaxLinearAcceleration();

  public double getTrackWidthX();

  public double getTrackWidthY();

  public default double getDriveBaseRadius() {
    return Math.hypot(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0);
  }

  public default double getMaxAngularSpeed() {
    return getMaxLinearSpeed() / getDriveBaseRadius();
  }
  /**
   * Derives angular acceleration from linear acceleration. This is a bad model because the MOI of a
   * robot is different from its mass.
   */
  public default double getMaxAngularAcceleration() {
    return getMaxLinearAcceleration() / getDriveBaseRadius();
  }
  /** Returns an array of module translations. */
  public default Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0),
      new Translation2d(getTrackWidthX() / 2.0, -getTrackWidthY() / 2.0),
      new Translation2d(-getTrackWidthX() / 2.0, getTrackWidthY() / 2.0),
      new Translation2d(-getTrackWidthX() / 2.0, -getTrackWidthY() / 2.0)
    };
  }

  public int getGyroID();

  public double getHeadingVelocityKP();

  public double getHeadingVoltageKP();

  public ModuleConstants getFrontLeftModule();

  public ModuleConstants getFrontRightModule();

  public ModuleConstants getBackLeftModule();

  public ModuleConstants getBackRightModule();

  public AprilTagFieldLayout getFieldTagLayout();

  public VisionConstants[] getVisionConstants();

  public default double getWheelRadiusMeters() {
    return Units.inchesToMeters(2.0);
  }

  public double getDriveGearRatio();

  public default double getDriveRotorToMeters() {
    return getDriveGearRatio() / (getWheelRadiusMeters() * 2 * Math.PI);
  }
  /** SDS Mk4 line default. */
  public default double getTurnGearRatio() {
    return 150.0 / 7.0;
  }
  /** Defaults to inverted ie Mk4i, Mk4n. */
  public default boolean getTurnMotorInverted() {
    return true;
  }

  public TalonFXConfiguration getDriveConfig();

  public TalonFXConfiguration getTurnConfig(int cancoderID);

  public CANcoderConfiguration getCancoderConfig(Rotation2d cancoderOffset);

  public default Matrix<N3, N1> getVisionPointBlankStdDevs() {
    return new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0.4, 0.4, 1});
  }

  public default double getVisionDistanceFactor() {
    return 0.5;
  }
}
