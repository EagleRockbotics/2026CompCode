// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.ChoreoTraj;
import frc.robot.autos.ChoreoTraj;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoHandlingSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;
import java.lang.constant.DirectMethodHandleDesc;
import java.lang.invoke.ConstantCallSite;
import java.lang.reflect.Method;
import java.lang.reflect.Type;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including`
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Pigeon2 m_gyro = new Pigeon2(Constants.kPigeonID);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final CommandXboxController driveStick = new CommandXboxController(
      Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController helperStick = new CommandXboxController(
      Constants.OperatorConstants.kHelperControllerPort);
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final AutoHandlingSubsystem m_autoHandler = new AutoHandlingSubsystem(m_drivetrain);
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(1, m_drivetrain, m_limelightSubsystem);
  private final Pair<Command, Supplier<SwerveRequest>> m_shooterPair = m_shooterSubsystem.shooterCommand(driveStick);
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Code copied from CTRE Swerve template
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity
  private double TeleopSpeedMultiplier = 0.1;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = driveStick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // TODO: Whenever you make a new subsytem, put it in this function.
    m_autoHandler.setupAutoReflection(this, m_drivetrain, m_autoHandler);
    m_autoHandler.publishChooser();
    
    CommandScheduler.getInstance().schedule(
      Commands.repeatingSequence(setRobotLimelightOrientationCommand(), addVisionMeasurementCommand())
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(m_drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    m_drivetrain.registerTelemetry(logger::telemeterize);

    m_shooterSubsystem.autoAimTeleopTrigger = joystick.rightBumper();
    m_shooterSubsystem.manualAimTeleopTrigger = joystick.leftBumper();
    m_shooterSubsystem.xAxis = () -> {return -joystick.getLeftX();};
    m_shooterSubsystem.yAxis = () -> {return -joystick.getLeftY();};

    m_elevatorSubsystem.backLeftButtonAxis = () -> {return helperStick.getLeftTriggerAxis();};
    m_elevatorSubsystem.backRightButtonAxis = () -> {return helperStick.getRightTriggerAxis();};
    m_elevatorSubsystem.backLeftButtonTrigger = helperStick.leftTrigger();
    m_elevatorSubsystem.backRightButtonTrigger = helperStick.rightTrigger();
    m_elevatorSubsystem.enableElevatorTrigger = driveStick.b();
    m_elevatorSubsystem.lowerElevatorTrigger = helperStick.povDown();
    m_elevatorSubsystem.raiseElevatorTrigger = helperStick.povUp();
    m_elevatorSubsystem.releaseSideServosTrigger = helperStick.a();
    m_elevatorSubsystem.runTopServoTrigger = helperStick.b();


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getTeleopCommand() {
    return Commands.parallel(
        m_drivetrain.applyRequest(this::getDriveRequest), m_shooterPair.getFirst());
  }

  private SwerveRequest getDriveRequest() {
          if (m_shooterSubsystem.autoAimTeleopTrigger.getAsBoolean()) {
            return m_shooterPair.getSecond().get();
          }
          return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * TeleopSpeedMultiplier) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * TeleopSpeedMultiplier) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
  }

  

  public Command getTestCommand() {
    return Commands.parallel(null);
  }

  public Command getAutoCommand() {
    m_drivetrain.resetThetaController();
    return m_autoHandler.getAutonomousCommand();
  }

  public Command resetGyro() {
    return Commands.runOnce(
      () -> {m_gyro.setYaw(0.0, 10);}
    );
  }

  public Command setRobotLimelightOrientationCommand() {
    return Commands.runOnce(
      () -> {m_limelightSubsystem.setRobotOrientation();}
    );
  }

  public Command addVisionMeasurementCommand() {
    return Commands.runOnce(
      () -> {
        m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(
          Constants.PoseEstimationConstants.kVisionXStdDev, // scale later
          Constants.PoseEstimationConstants.kVisionYStdDev, // scale later
          Constants.PoseEstimationConstants.kVisionHeadingStdDev
        ));
        LimelightHelpers.PoseEstimate poseEstimate = m_limelightSubsystem.getPoseEstimate();
        if (!m_limelightSubsystem.rejectUpdate()) {
          m_drivetrain.addVisionMeasurement(
             poseEstimate.pose,
             poseEstimate.timestampSeconds
          );
        }
      }
    );
  }

  public AutoRoutine driveTest(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("driveTest");
    AutoTrajectory driveFwdOneMeter = ChoreoTraj.NewPath.asAutoTraj(routine);
    routine.active().onTrue(Commands.parallel(
        Commands.sequence(driveFwdOneMeter.resetOdometry(), driveFwdOneMeter.cmd())));
    driveFwdOneMeter.done().onTrue(m_drivetrain.goToEndPose(driveFwdOneMeter));
    return routine;
  }

  public AutoRoutine turnTest(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("turnTest");
    AutoTrajectory spin = ChoreoTraj.Spinny.asAutoTraj(routine);
    routine.active().onTrue(Commands.sequence(spin.resetOdometry(), spin.cmd()));
    spin.done().onTrue(m_drivetrain.goToEndPose(spin));
    return routine;
  }

  public AutoRoutine movingTurnTest(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("movingTurnTest");
    AutoTrajectory traj = ChoreoTraj.MovingSpinny.asAutoTraj(routine);
    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    traj.done().onTrue(m_drivetrain.goToEndPose(traj));
    return routine;
  }
  
  public AutoRoutine visionHeadingStdevTuning(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("visionHeadingStdevTuning");
    AutoTrajectory traj = ChoreoTraj.HeadingStdevTuning.asAutoTraj(routine);
    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
    traj.done().onTrue(m_drivetrain.goToEndPose(traj));
    return routine;
  }
  

  public void publishAutoChooser() {
    this.m_autoHandler.publishChooser();
  }

  public void resetAutoRoutines() {
    this.m_autoHandler.resetRoutines();
  }

}
