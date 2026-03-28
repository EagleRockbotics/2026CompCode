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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import java.util.function.Supplier;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_driveMotor = new SparkFlex(Constants.ShooterConstants.kDriveMotorId, MotorType.kBrushless);
  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();
  private final CommandSwerveDrivetrain m_drivetrain;
  private final LimelightSubsystem m_limelightSubsystem;

  // SHOOTER MODE CONFIGURATION
  private final boolean forceLimelight = false;
  private final boolean useRobotVelocityCompensation = true; // TODO: set during testing & stuff
  private final boolean useShooterOffsetCompensation = true; // if true, shooter is aimed at hub; if false, robot is aimed at hub
  private final boolean useZippyZoomMath = false; // takes priority over useRobotVelocityCompensation (they're mutually exclusive)

  private final SparkMax m_indexerBeltMotor = new SparkMax(Constants.ShooterConstants.kIndexerBeltMotorId, MotorType.kBrushed);
  private final SparkMax m_indexerRollerMotor = new SparkMax(Constants.ShooterConstants.kIndexerRollerMotorId, MotorType.kBrushed);

  public Trigger autoAimTeleopTrigger = new Trigger(() -> {return false;});
  public Trigger manualAimTeleopTrigger = new Trigger(() -> {return false;});
  public Supplier<Double> xAxis = () -> {return 0d;};
  public Supplier<Double> yAxis = () -> {return 0d;};

  private final StructPublisher<Pose2d> m_targetAnglePublisher = NetworkTableInstance.getDefault().getStructTopic("Shooter/FacingTarget", Pose2d.struct).publish();
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
    
    SmartDashboard.putNumber("Shooter/Test Shooter RPM", 0);
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
    // Will return NaN if too close to Hub. Motor is n
    return Math.sqrt(((g*g)+(distance*distance))/(-2*Math.pow(Math.cos(theta),2)*(g*distance*Math.tan(theta)-(g*c))));
  }

  public double calculateRPMFromVelocity(double velocity) { // TODO: do this
    return 676.6*velocity+2866; // empirical, bound to change
  }

  public double calculateTargetAngle() { // calculates target angle while accounting for useShooterOffsetCompensation
    Translation2d hubPosition = getCurrentHubPosition();

    double targetAngle = Math.atan2(hubPosition.getY(), hubPosition.getX());
    if (useShooterOffsetCompensation) {
      targetAngle += Math.acos(Constants.ShooterConstants.kShooterDistanceFromCenter/hubPosition.getNorm()) - Math.PI/2;
    }
    return targetAngle;
  }

  public Command driveAtInputRPM() {
    return Commands.sequence(driveShooterCommand(() -> SmartDashboard.getNumber("Shooter/Test Shooter RPM", 0))).finallyDo(() -> m_driveMotor.set(0));
  }

  public Command pointingTest() {
    return Commands.run(() -> {
      
    });
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

  public Pair<Command, Supplier<SwerveRequest>> shooterCommand() {
    return new Pair<Command,Supplier<SwerveRequest>>(Commands.runOnce(() -> {
      autoAimTeleopTrigger.and(() -> !getRobotTooCloseToHub()).whileTrue(Commands.parallel(driveShooterCommand(this::getOutputRPM), Commands.run(() -> targetVelocityPublisher.set(calculateTargetVelocity(getHubDistance())))));
      manualAimTeleopTrigger.whileTrue(driveShooterCommand(() -> Constants.ShooterConstants.kPassRPM));
      autoAimTeleopTrigger.and(manualAimTeleopTrigger).whileFalse(shooterIdle());
  }), this::getAimRequest);
  }

  private double getOutputRPM() {
      Translation2d hubPosition = getCurrentHubPosition();
      double hubDistance = hubPosition.getNorm();
      double targetVelocity;
      if (useZippyZoomMath) {
        targetVelocity = zippyZoomMath(calculateTargetVelocity(hubDistance), hubPosition).getSecond();
      } else{
        targetVelocity = calculateTargetVelocity(hubDistance);
      }
      double RPMSetpoint = calculateRPMFromVelocity(targetVelocity);
      return RPMSetpoint;
  }

  private boolean getRobotTooCloseToHub() {
      Translation2d hubPosition = getCurrentHubPosition();
      double hubDistance = hubPosition.getNorm();
      return hubDistance < Constants.ShooterConstants.kMinRobotDistanceFromHub;
  }

  private double getHubDistance() {
      Translation2d hubPosition = getCurrentHubPosition();
      double hubDistance = hubPosition.getNorm();
      return hubDistance;
  }

  public Pair<Command, Supplier<Double>> autoShooterCommand() {
    return new Pair<Command, Supplier<Double>>(
      Commands.parallel(driveShooterCommand(this::getOutputRPM), Commands.run(() -> targetVelocityPublisher.set(calculateTargetVelocity(getHubDistance())))),
      this::calculateTargetAngle
    );
  }

  private Command driveShooterCommand(Supplier<Double> RPM) {
    return Commands.run(() -> {
      double rpm = -RPM.get();
    m_driveMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    rpmPublisher.set(this.m_driveMotor.getEncoder().getVelocity());
    System.out.println("fjdsk;ajflkdjalk;jdsaf");
    if (Math.abs(this.m_driveMotor.getEncoder().getVelocity() - rpm) < Constants.ShooterConstants.kMaxRPMOffsetBeforeShootFails) {
      m_indexerBeltMotor.set(Constants.ShooterConstants.kIndexerBeltPower);
      m_indexerRollerMotor.set(Constants.ShooterConstants.kIndexerRollerPower);
    } else {
      m_indexerBeltMotor.set(0);
      m_indexerRollerMotor.set(0);
    }
   }).finallyDo(() -> {m_indexerBeltMotor.set(0); m_indexerRollerMotor.set(0);}); 
  }


  double lastRPM = 0;
  private Command shooterIdle() {
    Timer offTime = new Timer();
    return Commands.sequence(Commands.runOnce(() -> offTime.restart()),
    Commands.runOnce(() -> lastRPM = m_driveMotor.getEncoder().getVelocity()), 
    Commands.run(() -> {
      m_driveMotor.getClosedLoopController().setSetpoint(lastRPM/(Constants.ShooterConstants.kIdleDropoff*offTime.get() + 1), ControlType.kVelocity); 
      rpmPublisher.set(m_driveMotor.getEncoder().getVelocity());}));
  }

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
    m_drivetrain.resetPose(currentPose);
    return new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromRadians(targetAngle))
      .withHeadingPID(Constants.ChoreoConstants.kP_theta, Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta)
      .withVelocityX(xAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed)
      .withVelocityY(yAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed);
  }

  public SwerveRequest getPointRequest() {
        Translation2d hubPosition = getCurrentHubPosition();
    Pose2d currentPose = getCurrentPose();
    double targetAngle;

    if (useZippyZoomMath) {
      targetAngle = zippyZoomMath(calculateTargetVelocity(hubPosition.getNorm()), hubPosition).getFirst();
    } else {
      targetAngle = calculateTargetAngle();
    }

    m_targetAnglePublisher.set(new Pose2d(currentPose.getTranslation(), Rotation2d.fromRadians(targetAngle)));
    m_drivetrain.resetPose(currentPose);
    return new SwerveRequest.FieldCentricFacingAngle()
      .withTargetDirection(Rotation2d.fromRadians(targetAngle))
      .withHeadingPID(Constants.ChoreoConstants.kP_theta, Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta)
      .withVelocityX(0)
      .withVelocityY(0);
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
