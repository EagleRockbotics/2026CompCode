// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
  private final XboxController m_controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort); // TODO: Switch to helper controller in the future

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
    
    Translation2d actualRelativeHubPosition = currentPosition.minus(Constants.FieldConstants.kHubPosition);
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

  public Command shooterCommand() {
    double RPMSetpoint = calculateRPMFromVelocity(calculateTargetVelocity(getLimelightDistanceFromHub()));
    return Commands.run(() -> {
      if (m_controller.getBButton()) {
        m_motorController.setSetpoint(RPMSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
      }
    });
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
