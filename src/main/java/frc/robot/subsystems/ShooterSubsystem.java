// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import java.util.function.Supplier;

import javax.sql.XAConnection;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_driveMotor = new SparkFlex(Constants.ShooterConstants.kDriveMotorId, MotorType.kBrushless);
  private final SparkClosedLoopController m_motorController = m_driveMotor.getClosedLoopController();
  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();
  private final CommandSwerveDrivetrain m_drivetrain;
  private final LimelightSubsystem m_limelightSubsystem;
  private final XboxController m_controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort); // TODO: Switch to helper controller in the future?

  // SHOOTER MODE CONFIGURATION
  private final boolean forceLimelight = true;
  private final boolean useRobotVelocityCompensation = true; // TODO: set during testing & stuff
  private final boolean useShooterOffsetCompensation = true; // if true, shooter is aimed at hub; if false, robot is aimed at hub
  private final boolean useZippyZoomMath = false; // takes priority over useRobotVelocityCompensation (they're mutually exclusive)

  private final SparkMax m_indexerBeltMotor = new SparkMax(Constants.ShooterConstants.kIndexerBeltMotorId, MotorType.kBrushed);
  private final SparkMax m_indexerRollerMotor = new SparkMax(Constants.ShooterConstants.kIndexerRollerMotorId, MotorType.kBrushed);

  public Trigger autoAimTeleopTrigger = new Trigger(() -> {return false;});
  public Trigger manualAimTeleopTrigger = new Trigger(() -> {return false;});
  public Supplier<Double> xAxis = () -> {return 0d;};
  public Supplier<Double> yAxis = () -> {return 0d;};

  private final StructPublisher m_targetAnglePublisher = NetworkTableInstance.getDefault().getStructTopic("Shooter/FacingTarget", Pose2d.struct).publish();
  private final DoublePublisher rpmPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/RPM").publish();
  private final DoublePublisher targetVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Shooter/Target Velocity").publish();

  @SuppressWarnings("removal")
  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    m_motorConfig.closedLoop.p(Constants.ShooterConstants.kP)
      .i(Constants.ShooterConstants.kI)
      .d(Constants.ShooterConstants.kD)
      .velocityFF(Constants.ShooterConstants.kF)
      .iZone(0.5);

    m_motorConfig.voltageCompensation(11);
    m_driveMotor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_drivetrain = drivetrain;
    m_limelightSubsystem = limelight;
  }

  public double getDistanceFromPose(Pose2d currentPose) {
    Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
    return currentPosition.getDistance(Constants.FieldConstants.kHubPosition); 
  }

  public Pose2d getCurrentPose() {
    if (forceLimelight) {
      return m_limelightSubsystem.getPoseEstimate().pose;
    } else {
      return m_drivetrain.getState().Pose;
    }
  }

  public Translation2d getCurrentPosition() {
    Pose2d currentPose = getCurrentPose();
    return new Translation2d(currentPose.getX(), currentPose.getY());
  }

  public Translation2d getEffectiveHubPosition() { // gets robot velocity-compensated hub position
    Translation2d actualHubPosition = Constants.FieldConstants.kHubPosition.minus(getCurrentPosition());
    double flightTime = calculateFlightTime(actualHubPosition.getNorm());
    ChassisSpeeds currentChassisSpeeds = m_drivetrain.getState().Speeds;

    Translation2d effectiveHubPosition = new Translation2d(
      flightTime*currentChassisSpeeds.vxMetersPerSecond + actualHubPosition.getX(),
      flightTime*currentChassisSpeeds.vyMetersPerSecond + actualHubPosition.getY()
    );
    return effectiveHubPosition;
  }

  @SuppressWarnings("unused")
  public Translation2d getCurrentHubPosition() {
    if (useRobotVelocityCompensation && !useZippyZoomMath) {
      return getEffectiveHubPosition();
    } else {
      return Constants.FieldConstants.kHubPosition.minus(getCurrentPosition()); 
    }
  }

  public double calculateFlightTime(double distance) {
    return distance/calculateTargetVelocity(distance);
  }

  public double calculateTargetVelocity(double distance) {
    double g = -9.81;
    double c = Constants.FieldConstants.kHubHeight-Constants.ShooterConstants.kShooterHeight;
    double theta = Constants.ShooterConstants.kShooterAngle;
    return Math.sqrt(((g*g)+(distance*distance))/(-2*Math.pow(Math.cos(theta),2)*(g*distance*Math.tan(theta)-(g*c))));
  }

  public double calculateRPMFromVelocity(double velocity) { // TODO: do this
    return 500 * velocity;
  }

  public double calculateTargetAngle() { // calculates target angle while accounting for useShooterOffsetCompensation
    Translation2d hubPosition = getCurrentHubPosition();

    double targetAngle = Math.atan(hubPosition.getY()/hubPosition.getX());
    if (useShooterOffsetCompensation) {
      targetAngle += Math.acos(Constants.ShooterConstants.kShooterDistanceFromCenter/hubPosition.getNorm()) - Math.signum(hubPosition.getX())*Constants.ShooterConstants.kShooterAngle;
    }
    return targetAngle;
  }

  //Returns robot angle offset and then shooter exit velocity (!!! not rpm). takes in exit velocity in m/s
  public Pair<Double, Double> zippyZoomMath(double desiredExitVelocity, Translation2d targetPoint) {
    Translation2d robotVelocity = new Translation2d(m_drivetrain.getState().Speeds.vxMetersPerSecond, m_drivetrain.getState().Speeds.vyMetersPerSecond);
    Translation2d robotToTarget = targetPoint.minus(m_drivetrain.getPose().getTranslation());
    Translation2d unitInTargetDirection = robotToTarget.div(robotToTarget.getNorm());
    Translation2d unitInOrthDirection = unitInTargetDirection.rotateBy(new Rotation2d(Math.PI/2));

    double robotVelocityTowardsPoint = robotVelocity.dot(unitInTargetDirection);
    double robotVelocityOrthToPoint = robotVelocity.dot(unitInOrthDirection)*Constants.SwerveConstants.kAirResistanceFactor;
    double newExitVelocity = Math.sqrt(Math.pow(desiredExitVelocity - robotVelocityTowardsPoint, 2) + Math.pow(robotVelocityOrthToPoint, 2));
    
    double angleOffset = -Math.asin(robotVelocityOrthToPoint/newExitVelocity);

    return new Pair<Double,Double>(angleOffset, newExitVelocity);
  }

  public Pair<Command, Supplier<SwerveRequest>> shooterCommand(CommandXboxController joystick) {
    return new Pair<Command,Supplier<SwerveRequest>>(Commands.run(() -> {
      Translation2d hubPosition = getCurrentHubPosition();
      double hubDistance = hubPosition.getNorm();
      double targetVelocity;
      if (useZippyZoomMath) {
        targetVelocity = zippyZoomMath(calculateTargetVelocity(hubDistance), hubPosition).getSecond();
      } else{
        targetVelocity = calculateTargetVelocity(hubDistance);
      }
      double RPMSetpoint = calculateRPMFromVelocity(targetVelocity);
      autoAimTeleopTrigger.and(() -> {return hubDistance < Constants.ShooterConstants.kMinRobotDistanceFromHub;}).whileTrue(Commands.parallel(driveShooterCommand(RPMSetpoint), Commands.run(() -> targetVelocityPublisher.set(calculateTargetVelocity(hubDistance)))));
      manualAimTeleopTrigger.whileTrue(driveShooterCommand(Constants.ShooterConstants.kPassRPM));
  }), this::getAimRequest);
  }

  private Command driveShooterCommand(double rpm) {
    return Commands.run(() -> {
    m_driveMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    rpmPublisher.set(this.m_driveMotor.getEncoder().getVelocity());
    if (Math.abs(this.m_driveMotor.getEncoder().getVelocity() - rpm) < Constants.ShooterConstants.kMaxRPMOffestBeforeShootFails) {
      m_indexerBeltMotor.set(Constants.ShooterConstants.kIndexerBeltPower);
      m_indexerRollerMotor.set(Constants.ShooterConstants.kIndexerRollerPower);
    } else {
      m_indexerBeltMotor.set(0);
      m_indexerRollerMotor.set(0);
    }
   }).finallyDo(() -> {m_indexerBeltMotor.set(0); m_indexerRollerMotor.set(0);});
  }

  @SuppressWarnings("unchecked")
  public SwerveRequest getAimRequest() {
    Translation2d hubPosition = getCurrentHubPosition();
    Pose2d currentPose = getCurrentPose();
    double targetAngle;

    if (useZippyZoomMath) {
      targetAngle = zippyZoomMath(calculateTargetVelocity(hubPosition.getNorm()), hubPosition).getFirst();
    } else {
      targetAngle = calculateTargetAngle();
    }

    m_targetAnglePublisher.set(new Pose2d(currentPose.getTranslation(), Rotation2d.fromRadians(targetAngle)));
    return new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromRadians(targetAngle))
      .withHeadingPID(Constants.ChoreoConstants.kP_theta, Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta)
      .withVelocityX(xAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed)
      .withVelocityY(yAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed);
  }

  @SuppressWarnings("unchecked")
  public SwerveRequest driveShooterFacingPoint(Translation2d targetPoint, Pose2d currentPose) {
        Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
        Translation2d relativeTargetPosition = currentPosition.plus(targetPoint);
        
        double x = relativeTargetPosition.getX()==0 ? 0.001 : relativeTargetPosition.getX();
        double y = relativeTargetPosition.getY();
        double targetAngle = Math.atan(y/x) + Math.acos(Constants.SwerveUtilConstants.kShooterDistanceFromCenter/Math.sqrt(Math.pow(x, 2)+Math.pow(y,2))) + Math.signum(x)*90;
        
        new Rotation2d();
        m_targetAnglePublisher.set(new Pose2d(currentPose.getTranslation(), Rotation2d.fromRadians(targetAngle)));
        return new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromRadians(targetAngle))
        .withHeadingPID(Constants.ChoreoConstants.kP_theta, Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta)
        .withVelocityX(xAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed)
        .withVelocityY(yAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed);
  }

  public SwerveRequest driveShooterFacingPoint(Translation2d targetPoint, double angleOffset, Pose2d currentPose) {
        Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
        Translation2d relativeTargetPosition = currentPosition.plus(targetPoint);
        
        double x = relativeTargetPosition.getX()==0 ? 0.001 : relativeTargetPosition.getX();
        double y = relativeTargetPosition.getY();
        double targetAngle = Math.atan(y/x) + Math.acos(Constants.SwerveUtilConstants.kShooterDistanceFromCenter/Math.sqrt(Math.pow(x, 2)+Math.pow(y,2))) + Math.signum(x)*90;
        
        m_targetAnglePublisher.set(new Pose2d(currentPose.getTranslation(), Rotation2d.fromRadians(targetAngle + angleOffset)));
        new Rotation2d();
        return new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromRadians(targetAngle + angleOffset))
        .withHeadingPID(Constants.ChoreoConstants.kP_theta, Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta)
        .withVelocityX(xAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed)
        .withVelocityY(yAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
        
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
