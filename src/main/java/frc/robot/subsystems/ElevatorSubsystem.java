// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SparkFlex m_elevatorMotor = new SparkFlex(Constants.ElevatorConstants.kElevatorMotorID, SparkLowLevel.MotorType.kBrushless);
  private final Servo m_bottomServo = new Servo(Constants.ElevatorConstants.kTopServoChannel);
  private final Servo m_topServo = new Servo(Constants.ElevatorConstants.kTopServoChannel);

  private final RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.kMaxVelocity, Constants.ElevatorConstants.kMaxAcceleration);
  private final ProfiledPIDController m_elevatorMotorController = new ProfiledPIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, m_constraints);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);


  public ElevatorSubsystem() {
    m_elevatorEncoder.setPosition(0);
    m_elevatorMotorController.setGoal(0);
    // If we want to raise servos during initialization
    // raiseTopServo();
    // raiseBottomServo();
  }
  public void lowerTopServo() {
      m_topServo.setAngle(Constants.ElevatorConstants.kServoDownAngle);
  }
  public void raiseTopServo() {
      m_topServo.setAngle(Constants.ElevatorConstants.kServoUpAngle);
  }
  public void lowerBottomServo() {
    m_bottomServo.setAngle(Constants.ElevatorConstants.kServoDownAngle);
  }
  public void raiseBottomServo() {
    m_bottomServo.setAngle(Constants.ElevatorConstants.kServoUpAngle);
  }
  public void raiseElevatorByOffset() {
    m_elevatorMotor.set(m_elevatorMotorController.calculate(m_elevatorEncoder.getPosition(), m_elevatorEncoder.getPosition()+Constants.ElevatorConstants.kElevatorGoalOffset));
  }
  public Command teleopCommand() {
    return run(
      () -> {

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
          /* one-time action goes here */
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
