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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SparkFlex m_motor = new SparkFlex(Constants.ElevatorConstants.kElevatorMotorID, SparkLowLevel.MotorType.kBrushless);
  private final Servo m_bottomServo = new Servo(Constants.ElevatorConstants.kTopServoChannel);
  private final Servo m_topServo = new Servo(Constants.ElevatorConstants.kTopServoChannel);

  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.kMaxVelocity, Constants.ElevatorConstants.kMaxAcceleration);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, m_constraints);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);

  private final XboxController m_helperStick;

  public ElevatorSubsystem() {
    m_encoder.setPosition(0);
    m_helperStick = new XboxController(Constants.OperatorConstants.kHelperControllerPort);
  }
  public Command teleopCommand() {
    return run(
      () -> {
        if(m_helperStick.getBButtonPressed()) {
          m_controller.setGoal(Constants.ElevatorConstants.kPosition1);
        } else{
          m_controller.setGoal(Constants.ElevatorConstants.kBottomPosition);
        }
        m_motor.setVoltage(m_controller.calculate(m_encoder.getPosition(), m_controller.getGoal())
        + m_feedforward.calculate(m_controller.getSetpoint().velocity));
      });
  }
  public Command lowerBottomServoCommand() {
    return runOnce(
      () -> {
        m_bottomServo.setAngle(Constants.ElevatorConstants.kServoDownAngle);
      });
  }
  public Command raiseBottomServoCommand() {
    return runOnce(
      () -> {
        m_bottomServo.setAngle(Constants.ElevatorConstants.kServoUpAngle);
      });
  }
  public Command lowerTopServoCommand() {
    return runOnce(
      () -> {
        m_topServo.setAngle(Constants.ElevatorConstants.kServoDownAngle);
      });
  }
  public Command raiseTopServoCommand() {
    return runOnce(
      () -> {
        m_topServo.setAngle(Constants.ElevatorConstants.kServoUpAngle);
      });
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
