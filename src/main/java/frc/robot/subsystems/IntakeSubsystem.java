// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;



public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_motorRightIntake = new SparkFlex(Constants.IntakeConstants.k_RightIntakeId, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex m_motorRightSpin= new SparkFlex(Constants.IntakeConstants.k_RightSpinId, SparkLowLevel.MotorType.kBrushless);
  private final PIDController m_pid  =  new PIDController(Constants.IntakeConstants.k_Kp, 0, Constants.IntakeConstants.k_Kd);

  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();
  private final RelativeEncoder m_RightEncoder = m_motorRightSpin.getEncoder();
  private final ArmFeedforward m_armFeed = new ArmFeedforward(Constants.IntakeConstants.k_Ks, Constants.IntakeConstants.k_Kg, Constants.IntakeConstants.k_Kv);
  public Trigger runIntakeTrigger = new Trigger(() -> {return false;});
  public Trigger reverseIntakeTrigger = new Trigger(() -> {return false;});
  public Trigger resetEncoderTrigger = new Trigger(() -> {return false;});

  private final DoublePublisher m_encoderPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Encoder").publish();
  private final DoublePublisher m_anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Angle").publish();


  public IntakeSubsystem() {
    m_RightEncoder.setPosition(0);
  }
  public Command runCommand() {
    return run(
        () -> {
          
          runIntakeTrigger.whileTrue(runIntakeCommand());
          runIntakeTrigger.whileFalse(returnToUpPositionCommand());
          resetEncoderTrigger.onTrue(resetEncoderCommand().onlyIf(() -> RobotModeTriggers.test().getAsBoolean()));
        });
  }

  //Creates the output needed for the motor spin to a certain radian
  // the arm feedforward is only dependent on the angle and speed we're trying to get the arm to.
  // Also you should never have numbers in your code with no description of what they are.
  // I can understand what "20" means in this function but putting it in a constant would make the code easier to read
  // The output of the encoder further needs to be converted into radians
  public double calculateMotorOutput(double desiredAngle) {
    double output = (m_pid.calculate((m_RightEncoder.getPosition() * (2*Math.PI)) / Constants.IntakeConstants.k_GearRatio, desiredAngle)
     + m_armFeed.calculate(desiredAngle, /* m_RightEncoder.getVelocity().getValueAsDouble() / 20 */ Constants.IntakeConstants.k_TargetVelocity));
     m_anglePublisher.set((m_RightEncoder.getPosition() * (2*Math.PI)/ Constants.IntakeConstants.k_GearRatio));
     m_encoderPublisher.set(m_RightEncoder.getPosition());
    return output;
  

  }

  public Command runIntakeCommand() {
    return Commands.run(() -> {
     int inversionFactor = 1;
      if (reverseIntakeTrigger.getAsBoolean()) {
        inversionFactor = -1;
      }
      m_motorRightSpin.set(calculateMotorOutput(Constants.IntakeConstants.k_TargetAngle));
      m_motorRightIntake.set(inversionFactor*Constants.IntakeConstants.k_IntakePower);
    });
  } 
  
  public Command returnToUpPositionCommand() {
    return Commands.run(() -> {
      m_motorRightSpin.set(calculateMotorOutput(Constants.IntakeConstants.k_UpAngle));
    });
  }
  
  public Command resetEncoderCommand() {
    return Commands.runOnce(() -> {
      m_RightEncoder.setPosition(0);
    });
  }

  public Command publishAngleCommand() {
    return Commands.run(() -> {
        m_anglePublisher.set((m_RightEncoder.getPosition() * (2*Math.PI)/ Constants.IntakeConstants.k_GearRatio));
        m_encoderPublisher.set(m_RightEncoder.getPosition());
    });
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
