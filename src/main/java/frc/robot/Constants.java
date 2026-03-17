// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. wow who would have thought? This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kLimelightName = "limelight-rock";
  public static final int kPigeonID = 14;

  public static class FieldConstants {
    public static final Translation2d kHubPosition = new Translation2d(0, 0); // TODO: Configure
    public static final double kHubHeight = 0; // Todo: height of hub opening
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kHelperControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double kDeadband = 0.1;
    public static final double kAngularAccelerationFactor = 4;
    public static final String kCurrentRobot = "comp2025";

    public static final double kAirResistanceFactor = 0;
  }

  public static class SwerveUtilConstants {
    public static double kShooterDistanceFromCenter = 1;
  }

  public static class ChoreoConstants {
    public static final PIDController xController = new PIDController(10, 0, 0);
    public static final PIDController yController = new PIDController(10, 0, 0);
    public static final double kP_theta = 15;
    public static final double kI_theta = 0;
    public static final double kD_theta = 0.01;
  }

  public static class PoseEstimationConstants {
    public static final double kOdometryXStdDev = 0.1;
    public static final double kOdometryYStdDev = 0.1;
    public static final double kOdometryHeadingStdDev = 0.001; // set to a small number because we have gyro for a reason
    public static final double kVisionXStdDev = 0.9;
    public static final double kVisionYStdDev = 0.9;
    public static final double kVisionHeadingStdDev = 67000; // set to a large number because we have gyro for a reason
  }

  public static class AutonomousConstants {
    public static final boolean kEnableAllianceFlipping = true;
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorID = 0;
    public static final int kLeftServoChannel = 0;
    public static final int kRightServoChannel = 0;
    public static final int kTopServoChannel = 0;
    public static final double kSideServoOutPosition = 0.5;
    public static final double kDownPosition = 0;
    public static final double kUpPosition = 1.0;
    public static final double kMaxVelocity = 10.0;
    public static final double kMaxAcceleration = 20.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}
