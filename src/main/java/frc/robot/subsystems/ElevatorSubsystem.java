// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

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

  public ElevatorSubsystem() {
    m_encoder.setPosition(0);
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command raiseElevatorCommand() {
    return Commands.run(() -> {
      if (backRightButtonAxis.get() > 0.05) {
        
      }
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
      if (elevatorEnabled) {
        m_motor.setVoltage(m_profiledController.calculate(m_encoder.getPosition()) + m_feedforward.calculate(m_profiledController.getSetpoint().velocity));
      }
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

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
