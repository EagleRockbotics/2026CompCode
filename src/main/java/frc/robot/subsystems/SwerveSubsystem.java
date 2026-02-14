// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.lang.StackWalker.Option;
import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {

  private SwerveDrive swerveDrive;
  private CommandXboxController input;
  private ProfiledPIDController thetaController;

  SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
  };

  StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Swerve/Observed", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> desiredPublisherFR = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Swerve/DesiredFR", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> desiredPublisherRR = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Swerve/DesiredRR", SwerveModuleState.struct).publish();
  StructPublisher<Rotation2d> gyroValue = NetworkTableInstance.getDefault().getStructTopic("Gyro", Rotation2d.struct)
      .publish();

  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem(File directory, CommandXboxController input) {
    this.input = input;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(3.8);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.zeroGyro();

    swerveDrive.useInternalFeedbackSensor();

    TrapezoidProfile.Constraints yagslConstraints = new TrapezoidProfile.Constraints(
        swerveDrive.getMaximumChassisAngularVelocity(),
        swerveDrive.getMaximumChassisAngularVelocity() * Constants.SwerveConstants.kAngularAccelerationFactor);
    thetaController = new ProfiledPIDController(Constants.ChoreoConstants.kP_theta,
        Constants.ChoreoConstants.kI_theta, Constants.ChoreoConstants.kD_theta, yagslConstraints);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command driveFieldRelative() {
    return run(
        () -> {
          ChassisSpeeds desiredSpeeds = new ChassisSpeeds(-this.input.getRawAxis(1), -this.input.getRawAxis(0),
              -this.input.getRawAxis(4));
          driveWrapper(desiredSpeeds);
       }).beforeStarting(() -> {
          SwerveModule[] modules = swerveDrive.getModules();
          // for (SwerveModule m : modules) {
          // SparkMax m_Motor = (SparkMax) m.getAngleMotor().getMotor();
          // CANcoder mEncoder = (CANcoder) m.getAbsoluteEncoder().getAbsoluteEncoder();
          // m_Motor.getAlternateEncoder().setPosition(mEncoder.getAbsolutePosition().refresh().getValueAsDouble()
          // *
          // 360);
          // }

        });

  }


  private void driveWrapper (ChassisSpeeds speeds) {
          this.swerveDrive.drive(speeds);
          this.swerveDrive.updateOdometry();
          swervePublisher.set(this.swerveDrive.getStates());
          desiredPublisherFR.set(this.swerveDrive.kinematics
              .toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, this.swerveDrive.getYaw())));
          desiredPublisherRR.set(this.swerveDrive.kinematics.toSwerveModuleStates(speeds));
          gyroValue.set(this.swerveDrive.getYaw());
          try {
            this.swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            this.swerveDrive.addVisionMeasurement(limelightPose().get().getFirst(), limelightPose().get().getSecond());
          } catch (Exception e) {

          }
  }

  public Optional<Pair<Pose2d, Double>> limelightPose () {

    var yaw = swerveDrive.getYaw().getDegrees();

    LimelightHelpers.SetRobotOrientation(Constants.kLimelightName, yaw, 0, 0, 0, 0, 0);
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.kLimelightName);
    if (swerveDrive.getGyro().getYawAngularVelocity().abs(DegreesPerSecond) > 360 || estimate.tagCount == 0) {
      return Optional.empty();
    }
    return Optional.of(new Pair<Pose2d,Double>(estimate.pose, estimate.timestampSeconds));
  }


  public Command updatedNT() {
    return run(() -> {
      swervePublisher.set(this.swerveDrive.getStates());
    }).beforeStarting(() -> {
      SwerveModule[] modules = swerveDrive.getModules();
      for (SwerveModule m : modules) {
        SparkMax m_Motor = (SparkMax) m.getAngleMotor().getMotor();
        CANcoder mEncoder = (CANcoder) m.getAbsoluteEncoder().getAbsoluteEncoder();
        m_Motor.getAlternateEncoder().setPosition(mEncoder.getAbsolutePosition().refresh().getValueAsDouble()
            *
            360);
      }

    });
  }

  public Command driveRobotRelative() {
    return run(
        () -> {
          ChassisSpeeds desiredSpeeds = new ChassisSpeeds(this.input.getRawAxis(1), this.input.getRawAxis(0),
              -this.input.getRawAxis(4));
          driveWrapper(desiredSpeeds);
          SwerveModule[] modules = swerveDrive.getModules();
          // for (SwerveModule m : modules) {
          // SparkMax m_Motor = (SparkMax) m.getAngleMotor().getMotor();
          // CANcoder mEncoder = (CANcoder) m.getAbsoluteEncoder().getAbsoluteEncoder();
          // m_Motor.getAlternateEncoder().setPosition(mEncoder.getAbsolutePosition().refresh().getValueAsDouble()
          // *
          // 360);
          // }

        });
  }

  public void followTrajectory(SwerveSample sample) {
    Pose2d pose = swerveDrive.getPose();
    ChassisSpeeds speeds = new ChassisSpeeds(
        sample.vx + Constants.ChoreoConstants.xController.calculate(pose.getX(), sample.x),
        sample.vy + Constants.ChoreoConstants.yController.calculate(pose.getY(), sample.y),
        sample.omega + thetaController.calculate(pose.getRotation().getRadians(), sample.heading));
    swerveDrive.driveFieldOriented(speeds);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d a) {
    swerveDrive.resetOdometry(a);
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
