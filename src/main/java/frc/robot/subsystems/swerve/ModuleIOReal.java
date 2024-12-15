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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.google.common.collect.ImmutableSet;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Registration;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType;
import java.util.Optional;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Swerve/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOReal implements ModuleIO {
  // Constants
  private static final boolean IS_TURN_MOTOR_INVERTED = true;

  // Constant so sim can access it
  public static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();

  static {
    // Current limits
    // TODO: Do we want to limit supply current?
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60.0;
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    // driveConfig.CurrentLimits.SupplyCurrentThreshold = 30.0;
    // driveConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    DRIVE_CONFIG.CurrentLimits.StatorCurrentLimit = 120.0;
    DRIVE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    DRIVE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    DRIVE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Sensor
    // Meters per second
    DRIVE_CONFIG.Feedback.SensorToMechanismRatio = Module.DRIVE_ROTOR_TO_METERS;
    // Voltage Controls Gains
    DRIVE_CONFIG.Slot0.kV = 2.381;
    DRIVE_CONFIG.Slot0.kA = 0.65;
    DRIVE_CONFIG.Slot0.kS = 0.04;
    DRIVE_CONFIG.Slot0.kP = 2.0;
    DRIVE_CONFIG.Slot0.kD = 0.2;

    // Current control gains
    DRIVE_CONFIG.Slot1.kV = 0.0;
    DRIVE_CONFIG.Slot1.kA = (9.37 / 483.0) / Module.DRIVE_ROTOR_TO_METERS; // 3.07135116146;
    DRIVE_CONFIG.Slot1.kS = 14.0;
    DRIVE_CONFIG.Slot1.kP = 100.0;
    DRIVE_CONFIG.Slot1.kD = 1.0;

    DRIVE_CONFIG.TorqueCurrent.TorqueNeutralDeadband = 10.0;

    DRIVE_CONFIG.MotionMagic.MotionMagicCruiseVelocity = SwerveSubsystem.MAX_LINEAR_SPEED;
    DRIVE_CONFIG.MotionMagic.MotionMagicAcceleration = SwerveSubsystem.MAX_LINEAR_ACCELERATION;
    // driveConfig.MotionMagic.MotionMagicJerk = SwerveSubsystem.MAX_LINEAR_ACCELERATION / 0.1;
  }

  private final ModuleConstants constants;

  // Hardware
  // Must be accessible to sim subclass
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Signals
  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveAppliedVolts;
  private final BaseStatusSignal driveCurrent;
  private final BaseStatusSignal driveSupplyCurrent;

  private final BaseStatusSignal turnAbsolutePosition;
  private final BaseStatusSignal turnPosition;
  private final BaseStatusSignal turnVelocity;
  private final BaseStatusSignal turnAppliedVolts;
  private final BaseStatusSignal turnCurrent;

  // Control modes
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC driveControlVelocity =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  // new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final double kAVoltsPerMeterPerSecondSquared;

  public ModuleIOReal(ModuleConstants constants) {
    this.constants = constants;

    driveTalon = new TalonFX(constants.driveID(), "canivore");
    turnTalon = new TalonFX(constants.turnID(), "canivore");
    cancoder = new CANcoder(constants.cancoderID(), "canivore");

    kAVoltsPerMeterPerSecondSquared = DRIVE_CONFIG.Slot0.kA;

    driveTalon.getConfigurator().apply(DRIVE_CONFIG);

    var turnConfig = new TalonFXConfiguration();
    // Current limits
    turnConfig.CurrentLimits.StatorCurrentLimit = Module.TURN_STATOR_CURRENT_LIMIT;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    turnConfig.MotorOutput.Inverted =
        IS_TURN_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Fused Cancoder
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.cancoderID();
    turnConfig.Feedback.RotorToSensorRatio = Module.TURN_GEAR_RATIO;
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.FeedbackRotorOffset =
        0.0; // Is this correct? Cancoder config should handle it
    // Controls Gains
    turnConfig.Slot0.kV = 2.7935;
    turnConfig.Slot0.kA = 0.031543;
    turnConfig.Slot0.kS = 0.28;
    turnConfig.Slot0.kP = 400.0;
    turnConfig.Slot0.kD = 0.68275;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 5500 / Module.TURN_GEAR_RATIO;
    turnConfig.MotionMagic.MotionMagicAcceleration = (5500 * 0.1) / Module.TURN_GEAR_RATIO;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turnTalon.getConfigurator().apply(turnConfig);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = constants.cancoderOffset().getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        IS_TURN_MOTOR_INVERTED
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    PhoenixOdometryThread.getInstance()
        .registerSignals(
            new Registration(
                driveTalon,
                Optional.of(constants),
                SignalType.DRIVE,
                ImmutableSet.of(drivePosition)),
            new Registration(
                turnTalon,
                Optional.of(constants),
                SignalType.STEER,
                ImmutableSet.of(turnPosition)));

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY_HZ, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(final ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(final double volts, final boolean focEnabled) {
    driveTalon.setControl(driveVoltage.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setDriveSetpoint(final double metersPerSecond, final double forceNewtons) {
    // Doesnt actually refresh drive velocity signal, but should be cached
    if (metersPerSecond == 0
        && forceNewtons == 0
        && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      setDriveVoltage(0.0);
    } else {
      driveTalon.setControl(
          driveControlVelocity
              .withVelocity(metersPerSecond)
              .withFeedForward(forceNewtons / DCMotor.getKrakenX60Foc(1).KtNMPerAmp));
    }
  }

  @Override
  public void setTurnSetpoint(final Rotation2d rotation) {
    turnTalon.setControl(turnPID.withPosition(rotation.getRotations()));
  }
}
