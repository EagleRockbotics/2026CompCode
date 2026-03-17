// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SparkFlex m_motor = new SparkFlex(Constants.ElevatorConstants.kElevatorMotorID, SparkLowLevel.MotorType.kBrushless);

  private final Servo m_leftServo = new Servo(Constants.ElevatorConstants.kLeftServoChannel);
  private final Servo m_rightServo = new Servo(Constants.ElevatorConstants.kRightServoChannel);
  private final Servo m_topServo = new Servo(Constants.ElevatorConstants.kTopServoChannel);

  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.kMaxVelocity, Constants.ElevatorConstants.kMaxAcceleration);
  private final ProfiledPIDController m_profiledController = new ProfiledPIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, m_constraints);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);

  private final boolean elevatorEnabled = true;

  public ElevatorSubsystem() {
    m_encoder.setPosition(0);
  }

  public Command raiseElevatorCommand() {
    return Commands.run(() -> {
      if (elevatorEnabled) {
        m_profiledController.setGoal(Constants.ElevatorConstants.kUpPosition);
      }
    });
  }
  public Command lowerElevatorCommand() {
    return Commands.run(() -> {
      if (elevatorEnabled) {
        m_profiledController.setGoal(Constants.ElevatorConstants.kDownPosition);
      }
    });
  }
  public Command releaseSideServosCommand() {
    return Commands.runOnce(() -> {
      if (elevatorEnabled) {
        m_leftServo.set(Constants.ElevatorConstants.kSideServoOutPosition);
        m_rightServo.set(Constants.ElevatorConstants.kSideServoOutPosition);
      }
    });
  }
  public Command runTopServoCommand() {
    return Commands.run(() -> {
      if (elevatorEnabled) {
        m_topServo.set(1); // because shog removed the encoder setting it to a nonzero value just makes it run
      }
    });
  }
  public Command runElevatorCommand() {
    return Commands.run(() -> {
      if (elevatorEnabled) {
        m_motor.setVoltage(m_profiledController.calculate(m_encoder.getPosition()) + m_feedforward.calculate(m_profiledController.getSetpoint().velocity));
      }
    });
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
