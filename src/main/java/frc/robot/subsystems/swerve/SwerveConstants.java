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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.vision.Vision.VisionConstants;

public abstract class SwerveConstants {
  private static boolean instantiated = false;
  private static final Alert multipleInstancesAlert =
      new Alert("Multiple Swerve Constants Files", AlertType.kError);

  public SwerveConstants() {
    if (instantiated) {
      multipleInstancesAlert.set(true);
    }
    instantiated = true;
  }

  // Drivebase Constants
  public abstract double getMaxLinearSpeed();

  public abstract double getMaxLinearAcceleration();

  public abstract double getTrackWidthX();

  public abstract double getTrackWidthY();

  public double getDriveBaseRadius() {
    return Math.hypot(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0);
  }

  public double getMaxAngularSpeed() {
    return getMaxLinearSpeed() / getDriveBaseRadius();
  }
  /**
   * Derives angular acceleration from linear acceleration. This is a bad model because the MOI of a
   * robot is different from its mass.
   */
  public double getMaxAngularAcceleration() {
    return getMaxLinearAcceleration() / getDriveBaseRadius();
  }
  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0),
      new Translation2d(getTrackWidthX() / 2.0, -getTrackWidthY() / 2.0),
      new Translation2d(-getTrackWidthX() / 2.0, getTrackWidthY() / 2.0),
      new Translation2d(-getTrackWidthX() / 2.0, -getTrackWidthY() / 2.0)
    };
  }

  public abstract int getGyroID();

  public abstract double getHeadingVelocityKP();

  public abstract double getHeadingVoltageKP();

  public abstract ModuleConstants getFrontLeftModule();

  public abstract ModuleConstants getFrontRightModule();

  public abstract ModuleConstants getBackLeftModule();

  public abstract ModuleConstants getBackRightModule();

  public abstract AprilTagFieldLayout getFieldTagLayout();

  public abstract VisionConstants[] getVisionConstants();

  public double getWheelRadiusMeters() {
    return Units.inchesToMeters(2.0);
  }

  public abstract double getDriveGearRatio();

  public double getDriveRotorToMeters() {
    return getDriveGearRatio() / (getWheelRadiusMeters() * 2 * Math.PI);
  }
  /** SDS Mk4 line default. */
  public double getTurnGearRatio() {
    return 150.0 / 7.0;
  }
  /** Defaults to inverted ie Mk4i, Mk4n. */
  public boolean getTurnMotorInverted() {
    return true;
  }

  public abstract TalonFXConfiguration getDriveConfig();

  public abstract TalonFXConfiguration getTurnConfig(int cancoderID);

  public abstract CANcoderConfiguration getCancoderConfig(Rotation2d cancoderOffset);

  public Matrix<N3, N1> getVisionPointBlankStdDevs() {
    return new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0.4, 0.4, 1});
  }

  public double getVisionDistanceFactor() {
    return 0.5;
  }
}
