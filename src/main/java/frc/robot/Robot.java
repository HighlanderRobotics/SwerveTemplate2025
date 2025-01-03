// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.swerve.BansheeSwerveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOReal;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.Tracer;
import java.util.HashMap;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotHardware {
    BANSHEE(new BansheeSwerveConstants()),
    ALPHA(null), // TODO Add swerve constants as appropriate
    COMP(null); // TODO Add swerve constants as appropriate

    public final SwerveConstants swerveConstants;

    private RobotHardware(SwerveConstants swerveConstants) {
      this.swerveConstants = swerveConstants;
    }
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;
  // For replay to work properly this should match the hardware used in the log
  public static final RobotHardware ROBOT_HARDWARE = RobotHardware.BANSHEE;

  private final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);

  private final SwerveSubsystem swerve =
      new SwerveSubsystem(
          ROBOT_HARDWARE.swerveConstants,
          ROBOT_TYPE == RobotType.REAL
              ? new GyroIOPigeon2(ROBOT_HARDWARE.swerveConstants.getGyroID())
              : new GyroIO() {
                // Blank impl in sim.
                @Override
                public void updateInputs(GyroIOInputs inputs) {}

                @Override
                public void setYaw(Rotation2d yaw) {}
              },
          Stream.of(ROBOT_HARDWARE.swerveConstants.getVisionConstants())
              .map(
                  (constants) ->
                      ROBOT_TYPE == RobotType.REAL
                          ? new VisionIOReal(constants)
                          : new VisionIOSim(constants))
              .toArray(VisionIO[]::new),
          ROBOT_TYPE == RobotType.REAL
              ? new ModuleIO[] {
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getFrontLeftModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getFrontRightModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getBackLeftModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOReal(
                    ROBOT_HARDWARE.swerveConstants.getBackRightModule(),
                    ROBOT_HARDWARE.swerveConstants)
              }
              : new ModuleIO[] {
                new ModuleIOSim(
                    ROBOT_HARDWARE.swerveConstants.getFrontLeftModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOSim(
                    ROBOT_HARDWARE.swerveConstants.getFrontRightModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOSim(
                    ROBOT_HARDWARE.swerveConstants.getBackLeftModule(),
                    ROBOT_HARDWARE.swerveConstants),
                new ModuleIOSim(
                    ROBOT_HARDWARE.swerveConstants.getBackRightModule(),
                    ROBOT_HARDWARE.swerveConstants)
              },
          PhoenixOdometryThread.getInstance());

  private final Autos autos;
  // Could make this cache like Choreo's AutoChooser, but thats more work and Choreo's default
  // option isn't akit friendly
  // Main benefit to that is reducing startup time, which idt we care about too much
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

  public Robot() {
    SignalLogger.enableAutoLogging(false);
    RobotController.setBrownoutVoltage(6.0);
    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "Comp2025");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", ROBOT_TYPE.toString());
    Logger.recordMetadata("Robot Hardware", ROBOT_HARDWARE.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    initializeTracerLogging();

    switch (ROBOT_TYPE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    SignalLogger.setPath("/media/sda1/");

    autos = new Autos(swerve);
    autoChooser.addDefaultOption("None", autos.getNoneAuto());
    
    // Run auto when auto starts. Matches Choreolib's defer impl
    RobotModeTriggers.autonomous().whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    // Default Commands

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));

    swerve.setDefaultCommand(
        swerve.driveTeleop(
            () ->
                new ChassisSpeeds(
                    modifyJoystick(driver.getLeftY())
                        * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                    modifyJoystick(driver.getLeftX())
                        * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
                    modifyJoystick(driver.getRightX())
                        * ROBOT_HARDWARE.swerveConstants.getMaxAngularSpeed())));
  }

  /** Scales a joystick value for teleop driving */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.02);
  }

  @Override
  public void loopFunc() {
    Tracer.trace("Robot/LoopFunc", super::loopFunc);
  }

  private void initializeTracerLogging() {
    HashMap<String, Integer> commandCounts = new HashMap<>();
    final BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "Commands/CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
              active.booleanValue());
          Logger.recordOutput("Commands/CommandsAll/" + name, count > 0);
        };

    var scheduler = CommandScheduler.getInstance();

    scheduler.onCommandInitialize(c -> logCommandFunction.accept(c, true));
    scheduler.onCommandFinish(c -> logCommandFunction.accept(c, false));
    scheduler.onCommandInterrupt(c -> logCommandFunction.accept(c, false));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
