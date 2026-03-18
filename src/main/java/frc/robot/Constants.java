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
    public static final int kTopServoChannel = 0;
    public static final int kBottomServoChannel = 0;

    public static final double kMaxVelocity = 0;
    public static final double kMaxAcceleration = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kG = 0;

    public static final double kPosition1 = 0;
    public static final double kBottomPosition = 0;

    public static final double kServoDownAngle = 0;
    public static final double kServoUpAngle = 0;



  }

  public static class ShooterConstants {
    public static final double kShooterHeight = 0; // TODO: height of ball as it leaves shooter
    public static final double kShooterAngle = 0; // TODO: angle
    public static final int kDriveMotorId = 0; // TODO: Configure
    public static double kShooterDistanceFromCenter = 1;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;
    
    public static final int kIndexerBeltMotorId = 0;
    public static final int kIndexerRollerMotorId = 0;

    public static final double kIndexerBeltPower = 0.9;
    public static final double kIndexerRollerPower = -0.9;

    public static final double kMaxRPMOffestBeforeShootFails = 150; //guess. please change
  }

  public static class IntakeConstants {
    public static final double kdt = 0.02;
    //Motor ID's
    public static final int k_RightIntakeId = 0;
    public static final int k_LeftIntakeId = 0;
    public static final int k_RightSpinId = 0;
    public static final int k_LeftSpinId = 0;
    //encoder ID's
    public static final int k_RightEncoderId = 0;
    public static final int k_LeftEncoderId = 0;
    //Motor Power
    public static final double k_IntakePower = 0;
    public static final double k_SpinPower = 0;
    //PID Constants
    public static final double k_Kp = 0;
    public static final double k_Kd = 0;
    //Arm Feedwater Constants
    public static final double k_Ks = 0;
    public static final double k_Kg = 0;
    public static final double k_Kv = 0;

    public static final double k_EncoderThreshold = 0;

    public static final double k_TargetAngle = 90;
    }
}
