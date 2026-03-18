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
  private final int mode;
  private final boolean aimWithRobotVelocity = false; // TODO: set during testing & stuff

  private final SparkMax m_indexerBeltMotor = new SparkMax(Constants.ShooterConstants.kIndexerBeltMotorId, MotorType.kBrushed);
  private final SparkMax m_indexerRollerMotor = new SparkMax(Constants.ShooterConstants.kIndexerRollerMotorId, MotorType.kBrushed);

  public Trigger autoAimTeleopTrigger = new Trigger(() -> {return false;});
  public Trigger manualAimTeleopTrigger = new Trigger(() -> {return false;});
  public Supplier<Double> xAxis = () -> {return 0d;};
  public Supplier<Double> yAxis = () -> {return 0d;};

  
  // mode 0: manual aim, uses limelight for pose calculation

  // mode 1: aims robot directly at hub, uses combined vison & odometry measurement for pose calculation
  // mode 2: aims shooter at hub (accounting for robot motion if aimWithRobotVelocity), uses combined measurements

  @SuppressWarnings("removal")
  public ShooterSubsystem(int _mode, CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    mode = _mode;
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

  public double getLimelightDistanceFromHub() {
    Pose2d currentPose = m_limelightSubsystem.getPoseEstimate().pose;
    Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
    return currentPosition.getDistance(Constants.FieldConstants.kHubPosition);
  }

  public double getCombinedDistanceFromHub() {
    Pose2d currentPose = m_drivetrain.getState().Pose;
    Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
    return currentPosition.getDistance(Constants.FieldConstants.kHubPosition);
  }

  public double getEffectiveDistanceFromHub() {
    Pose2d currentPose = m_drivetrain.getState().Pose;
    Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
    
    Translation2d actualRelativeHubPosition = Constants.FieldConstants.kHubPosition.minus(currentPosition);
    double actualHubDistance = actualRelativeHubPosition.getNorm();
    double flightTime = calculateFlightTime(actualHubDistance);
    
    ChassisSpeeds currentChassisSpeeds = m_drivetrain.getState().Speeds;
    Translation2d effectiveRelativeHubPosition = new Translation2d(
      flightTime*currentChassisSpeeds.vxMetersPerSecond*actualRelativeHubPosition.getX(),
      flightTime*currentChassisSpeeds.vyMetersPerSecond*actualRelativeHubPosition.getY()
    );
    return effectiveRelativeHubPosition.getNorm();
  }

  public double calculateFlightTime(double distance) {
    return Math.sqrt((2/9.81)*(Constants.FieldConstants.kHubHeight - Constants.ShooterConstants.kShooterHeight - distance*Math.toRadians(Constants.ShooterConstants.kShooterAngle)));
  }

  public double calculateTargetVelocity(double distance) { // i will not put gravity as a constant in the constant file shut the frick up lazar
    return distance/(Math.cos(Constants.ShooterConstants.kShooterAngle)*Math.sqrt((2/9.81)*(Constants.FieldConstants.kHubHeight-Constants.ShooterConstants.kShooterHeight-(distance*Math.tan(Constants.ShooterConstants.kShooterAngle)))));
  }

  public double calculateRPMFromVelocity(double velocity) { // TODO: do this
    return velocity;
  }

  public Pair<Command, Supplier<SwerveRequest>> shooterCommand(CommandXboxController joystick) {
    return new Pair<Command,Supplier<SwerveRequest>>(Commands.run(() -> {
      double hubDistance;
    switch (mode) {
      case 2:
        hubDistance = getEffectiveDistanceFromHub();
        break;
      case 1:
        hubDistance = getCombinedDistanceFromHub();
        break;
      default:
        hubDistance = getLimelightDistanceFromHub();
        break;
    }
    double RPMSetpoint = (mode == 4 || mode == 5) ? m_drivetrain.zippyZoomMath(calculateTargetVelocity(hubDistance), Constants.FieldConstants.kHubPosition).getSecond() : calculateRPMFromVelocity(calculateTargetVelocity(hubDistance));
    autoAimTeleopTrigger.or(manualAimTeleopTrigger).whileTrue(driveShooterCommand(RPMSetpoint));
  }), this::getAimRequest);
    }

  private Command driveShooterCommand(double rpm) {
    return Commands.run(() -> {
    m_driveMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    if (Math.abs(this.m_driveMotor.getEncoder().getVelocity() - rpm) < Constants.ShooterConstants.kMaxRPMOffestBeforeShootFails) {
      m_indexerBeltMotor.set(Constants.ShooterConstants.kIndexerBeltPower);
      m_indexerRollerMotor.set(Constants.ShooterConstants.kIndexerRollerPower);
    }
   }).finallyDo(() -> {m_indexerBeltMotor.set(0); m_indexerRollerMotor.set(0);});
  }

  


  @SuppressWarnings("static-access")
  public SwerveRequest getAimRequest() {
    Pose2d currentPose = m_drivetrain.getState().Pose;
    Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
    Translation2d actualRelativeHubPosition = Constants.FieldConstants.kHubPosition.minus(currentPosition);

    double x = actualRelativeHubPosition.getX();
    double y = actualRelativeHubPosition.getY();
    double targetAngle;
    double hubDistance;

    switch (mode) {
      case 2:
        hubDistance = getEffectiveDistanceFromHub();
        break;
      case 1:
        hubDistance = getCombinedDistanceFromHub();
        break;
      default:
        hubDistance = getLimelightDistanceFromHub();
        break;
    }    

    switch (mode) {
      case 4:
        return driveShooterFacingPoint(actualRelativeHubPosition, m_drivetrain.zippyZoomMath(calculateTargetVelocity(hubDistance), Constants.FieldConstants.kHubPosition).getFirst() ,xAxis, yAxis);
      case 3:
          double flightTime = calculateFlightTime(actualRelativeHubPosition.getNorm());
          ChassisSpeeds currentChassisSpeeds = m_drivetrain.getState().Speeds;
          Translation2d effectiveRelativeHubPosition = new Translation2d(
            flightTime*currentChassisSpeeds.vxMetersPerSecond*actualRelativeHubPosition.getX(),
            flightTime*currentChassisSpeeds.vyMetersPerSecond*actualRelativeHubPosition.getY()
          );
        return driveShooterFacingPoint(effectiveRelativeHubPosition, xAxis, yAxis);
      case 2:
        if (aimWithRobotVelocity) {
          double old_flightTime = calculateFlightTime(actualRelativeHubPosition.getNorm());
          ChassisSpeeds old_currentChassisSpeeds = m_drivetrain.getState().Speeds;
          Translation2d old_effectiveRelativeHubPosition = new Translation2d(
            flightTime*currentChassisSpeeds.vxMetersPerSecond*actualRelativeHubPosition.getX(),
            flightTime*currentChassisSpeeds.vyMetersPerSecond*actualRelativeHubPosition.getY()
          );
          x = old_effectiveRelativeHubPosition.getX();
          y = old_effectiveRelativeHubPosition.getY();
        }
        targetAngle = Math.toDegrees(Math.atan(y/x) + Math.acos(Constants.ShooterConstants.kShooterDistanceFromCenter/(Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2)))) - (Math.signum(x)*Math.PI/2));
        break;
      case 1:
        targetAngle = Math.toDegrees(Math.atan(y/x));
        break;
      default:
        targetAngle = 0;
    }
    return new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(new Rotation2d().fromDegrees(targetAngle))
            .withHeadingPID(Constants.ChoreoConstants.kP_theta, Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta)
            .withVelocityX(xAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed)
            .withVelocityY(yAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed);
  }

  public SwerveRequest driveShooterFacingPoint(Translation2d targetPoint, Supplier<Double> xAxis, Supplier<Double> yAxis) {
        Pose2d currentPose = m_drivetrain.getPose();
        Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
        Translation2d relativeTargetPosition = currentPosition.plus(targetPoint);
        
        double x = relativeTargetPosition.getX()==0 ? 0.001 : relativeTargetPosition.getX();
        double y = relativeTargetPosition.getY();
        double targetAngle = Math.atan(y/x) + Math.acos(Constants.SwerveUtilConstants.kShooterDistanceFromCenter/Math.sqrt(Math.pow(x, 2)+Math.pow(y,2))) + Math.signum(x)*90;
        
        new Rotation2d();
        return new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(targetAngle))
        .withHeadingPID(Constants.ChoreoConstants.kP_theta, Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta)
        .withVelocityX(xAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed)
        .withVelocityY(yAxis.get()*Constants.ShooterConstants.kMaxScoringRobotSpeed);
  }

  public SwerveRequest driveShooterFacingPoint(Translation2d targetPoint, double angleOffset, Supplier<Double> xAxis, Supplier<Double> yAxis) {
        Pose2d currentPose = m_drivetrain.getPose();
        Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
        Translation2d relativeTargetPosition = currentPosition.plus(targetPoint);
        
        double x = relativeTargetPosition.getX()==0 ? 0.001 : relativeTargetPosition.getX();
        double y = relativeTargetPosition.getY();
        double targetAngle = Math.atan(y/x) + Math.acos(Constants.SwerveUtilConstants.kShooterDistanceFromCenter/Math.sqrt(Math.pow(x, 2)+Math.pow(y,2))) + Math.signum(x)*90;
        
        new Rotation2d();
        return new SwerveRequest.FieldCentricFacingAngle()
        .withTargetDirection(Rotation2d.fromDegrees(targetAngle))
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
