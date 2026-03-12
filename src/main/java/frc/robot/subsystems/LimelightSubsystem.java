// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Micro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rock");
  private final StructPublisher<Pose2d> posePublisher;
  private final Pigeon2 m_gyro;

  public LimelightSubsystem() {
    posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Limelight/Pose", Pose2d.struct).publish();
    m_gyro = new Pigeon2(Constants.kPigeonID);
  }

  public Command sendRobotOrientationCommand(Pigeon2 gyro) {
    double[] robotorientation = { gyro.getYaw().getValueAsDouble(), gyro.getAngularVelocityZWorld().getValueAsDouble(),
        0, 0, 0, 0 };
    return run(() -> {
      table.getEntry("robot_orientation_set").setDoubleArray(robotorientation);
      try {
        posePublisher.set(getRobotPose());
      } catch (Exception e) {
        e.printStackTrace();
      }
    });

  }

  public Pose2d getRobotPose() {

    double[] value = table.getEntry("botpose_orb_wpiblue").getDoubleArray((double[]) null);
    if (value == (double[]) null) {
      return new Pose2d(1, 0, new Rotation2d(0));
    }
    var out = new Pose2d(new Translation2d(value[0], value[1]), new Rotation2d(value[5] * Math.PI / 180));
    return out;

  }

  public double getLatency() {
    double[] value = table.getEntry("botpose_orb_wpiblue").getDoubleArray((double[]) null);
    return value[6];
  }

  public boolean setFuducialFilter(double[] idFilters) {
    return table.getEntry("fiducial_id_filters_set").setDoubleArray(idFilters);
  }

  public Command poseCommand() {
    return sendRobotOrientationCommand(m_gyro);
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rock");
  }

  public void setRobotOrientation() {
    LimelightHelpers.SetRobotOrientation("limelight-rock", 
      m_gyro.getYaw().getValueAsDouble(), m_gyro.getAngularVelocityZDevice().getValueAsDouble(), 
      m_gyro.getPitch().getValueAsDouble(), m_gyro.getAngularVelocityYDevice().getValueAsDouble(), 
      m_gyro.getRoll().getValueAsDouble(), m_gyro.getAngularVelocityXDevice().getValueAsDouble());
  }

  public Boolean rejectUpdate() {
    Boolean doRejectUpdate = false;
    if (Math.abs(m_gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360) {
      doRejectUpdate = true;
    }
    if (LimelightHelpers.getTargetCount("limelight-rock") == 0) {
      doRejectUpdate = true;
    }
    return doRejectUpdate;
  }
}