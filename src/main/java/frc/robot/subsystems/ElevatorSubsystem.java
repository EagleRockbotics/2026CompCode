// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANrange distanceSensor = new CANrange(0);

  private final SparkFlex m_motor = new SparkFlex(Constants.ElevatorConstants.kElevatorMotorID, SparkLowLevel.MotorType.kBrushless);
  private final SparkBaseConfig m_motorConfig = new SparkFlexConfig().idleMode(IdleMode.kBrake);

  private final Servo m_leftServo = new Servo(Constants.ElevatorConstants.kLeftServoChannel);
  private final Servo m_rightServo = new Servo(Constants.ElevatorConstants.kRightServoChannel);
  private final Servo m_topServo = new Servo(Constants.ElevatorConstants.kTopServoChannel);

  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.kMaxVelocity, Constants.ElevatorConstants.kMaxAcceleration);
  private final ProfiledPIDController m_profiledController = new ProfiledPIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, m_constraints);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);

  private boolean elevatorEnabled = true;
  private boolean sideServosReleased = false;

  public Trigger raiseElevatorTrigger = new Trigger(() -> {return false;});
  public Trigger lowerElevatorTrigger = new Trigger(() -> {return false;});
  public Trigger releaseSideServosTrigger = new Trigger(() -> {return false;});
  public Trigger runTopServoTrigger = new Trigger(() -> {return false;});
  public Trigger enableElevatorTrigger = new Trigger(() -> {return false;});

  public Trigger backLeftButtonTrigger = new Trigger(() -> {return false;});
  public Trigger backRightButtonTrigger = new Trigger(() -> {return false;});
  public Supplier<Double> backLeftButtonAxis = () -> {return 0d;};
  public Supplier<Double> backRightButtonAxis = () -> {return 0d;};

  public boolean useDistanceSensor = true;

  private final DoublePublisher encoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Elevator/Encoder").publish();
  private final DoublePublisher PIDOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Elevator/PIDOutput").publish();
  private final DoublePublisher feedforwardOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Elevator/FeedforwardOutput").publish();

  public ElevatorSubsystem() {
    m_encoder.setPosition(0);
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command raiseElevatorCommand() {
    return Commands.run(() -> {
      m_profiledController.setGoal(Constants.ElevatorConstants.kUpPosition);
    });
  }
  public Command lowerElevatorCommand() {
    return Commands.run(() -> {
      m_profiledController.setGoal(Constants.ElevatorConstants.kDownPosition);
    });
  }
  public Command releaseSideServosCommand() {
    return Commands.runOnce(() -> {
      if (!sideServosReleased) {
        m_leftServo.set(Constants.ElevatorConstants.kSideServoOutPosition);
        m_rightServo.set(Constants.ElevatorConstants.kSideServoOutPosition);
        sideServosReleased = true;
      } else {
        System.out.println("Elevator Subsystem: Side servos already released.");
      }
    });
  }
  public Command runTopServoCommand() {
    return Commands.run(() -> {
      m_topServo.set(1); // because shog removed the encoder setting it to a nonzero value just makes it run
    });
  }
  public Command setElevatorVoltageCommand() {
    return Commands.run(() -> {
      double PIDOutput = m_profiledController.calculate(m_encoder.getPosition());
      double feedforwardOutput = m_feedforward.calculate(m_profiledController.getSetpoint().velocity);
      m_motor.setVoltage(PIDOutput + feedforwardOutput);

      encoderPublisher.set(m_encoder.getPosition());
      PIDOutputPublisher.set(PIDOutput);
      feedforwardOutputPublisher.set(feedforwardOutput);
    });
  }
  public Command enableElevatorCommand() {
    return Commands.runOnce(() -> {
      elevatorEnabled = true;
    });
  }
  public Command disableElevatorCommand() {
    return Commands.runOnce(() -> {
      elevatorEnabled = false;
    });
  }

  public Command elevatorCommand() {
    return Commands.runOnce(() -> {
      enableElevatorTrigger.whileTrue(Commands.run(() -> { // this is dogshit
        raiseElevatorTrigger.whileTrue(raiseElevatorCommand().onlyWhile(backLeftButtonTrigger.or(backRightButtonTrigger).negate()));
        lowerElevatorTrigger.whileTrue(lowerElevatorCommand().onlyWhile(backLeftButtonTrigger.or(backRightButtonTrigger).negate()));
        backLeftButtonTrigger.whileTrue(Commands.run(() -> {
          m_profiledController.setConstraints(new TrapezoidProfile.Constraints(
            Math.min(Constants.ElevatorConstants.kMaxVelocity*backLeftButtonAxis.get(),Constants.ElevatorConstants.kMaxVelocity), Constants.ElevatorConstants.kMaxAcceleration
          ));
          lowerElevatorCommand();
        }).onlyIf(backRightButtonTrigger.negate()));
        backRightButtonTrigger.whileTrue(Commands.run(() -> {
          m_profiledController.setConstraints(new TrapezoidProfile.Constraints(
            Math.min(Constants.ElevatorConstants.kMaxVelocity*backRightButtonAxis.get(),Constants.ElevatorConstants.kMaxVelocity), Constants.ElevatorConstants.kMaxAcceleration
          ));
          raiseElevatorCommand();
        }).onlyIf(backLeftButtonTrigger.negate()));
        releaseSideServosTrigger.onTrue(releaseSideServosCommand());
        runTopServoTrigger.onTrue(runTopServoCommand());
      }));
    });
  }

  public Command climbStageSequence() {
    return Commands.sequence(
      raiseElevatorCommand(), lowerElevatorCommand()
    );
  }

  public Command autoTeleopClimbSequence() {
    return Commands.sequence(
      runTopServoCommand().withTimeout(Constants.AutonomousConstants.kAutoElevatorTopServoTimeout),
      raiseElevatorCommand(),
      releaseSideServosCommand(),
      lowerElevatorCommand(),
      raiseElevatorCommand(),
      lowerElevatorCommand(),
      raiseElevatorCommand(),
      lowerElevatorCommand()
    );
  }

  public Command autonomousClimbSequence() {
    return Commands.sequence(
      runTopServoCommand().withTimeout(Constants.AutonomousConstants.kAutoElevatorTopServoTimeout),
      raiseElevatorCommand(),
      lowerElevatorCommand()
    );
  }

  public Command teleopAlignWithLadder(CommandSwerveDrivetrain drivetrain) {
    Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d targetPose = currentPose.getTranslation().getDistance(Constants.FieldConstants.kLeftLadderPose.getTranslation()) <
      currentPose.getTranslation().getDistance(Constants.FieldConstants.kRightLadderPose.getTranslation()) ?
      Constants.FieldConstants.kLeftLadderPose.plus(new Transform2d(new Translation2d(Constants.ElevatorConstants.kElevatorPositionFrontOffset, 0), new Rotation2d(Math.PI))) : 
      Constants.FieldConstants.kRightLadderPose.plus(new Transform2d(new Translation2d(Constants.ElevatorConstants.kElevatorPositionFrontOffset, 0), new Rotation2d(Math.PI)));
    
    return drivetrain.moveToPose(targetPose);
  }

  public Command moveToLadder(CommandSwerveDrivetrain drivetrain) {
    Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d targetPose = currentPose.getTranslation().getDistance(Constants.FieldConstants.kLeftLadderPose.getTranslation()) <
      currentPose.getTranslation().getDistance(Constants.FieldConstants.kRightLadderPose.getTranslation()) ?
      Constants.FieldConstants.kLeftLadderPose : Constants.FieldConstants.kRightLadderPose;
    
    if (useDistanceSensor) {
      return drivetrain.moveToDistanceSensorPoint(distanceSensor, Constants.FieldConstants.kLeftLadderPose.getX() + Constants.SwerveConstants.kRobotLength/2, targetPose);
    } else {
      return drivetrain.moveToPose(targetPose);
    }
  }

  @Override
  public void periodic() {
    encoderPublisher.set(m_encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
