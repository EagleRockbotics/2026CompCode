// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;



public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_motorRightIntake = new SparkFlex(Constants.IntakeConstants.k_RightIntakeId, SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex m_motorRightSpin= new SparkFlex(Constants.IntakeConstants.k_RightSpinId, SparkLowLevel.MotorType.kBrushless);
  private final PIDController m_pid  =  new PIDController(Constants.IntakeConstants.k_Kp, 0, Constants.IntakeConstants.k_Kd);

  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();
  private final CANcoder m_RightEncoder = new CANcoder(Constants.IntakeConstants.k_RightEncoderId);
  private final ArmFeedforward m_armFeed = new ArmFeedforward(Constants.IntakeConstants.k_Ks, Constants.IntakeConstants.k_Kg, Constants.IntakeConstants.k_Kv);

  public Trigger runIntakeTrigger = new Trigger(() -> {return false;});
  public Trigger reverseIntakeTrigger = new Trigger(() -> {return false;});

  public IntakeSubsystem() {
    // m_RightEncoder.set(0.0d);
  }

  public Command runCommand() {
    return run(
        () -> {
          runIntakeTrigger.whileTrue(runIntakeCommand());
        });
  }

  //Creates the output needed for the motor spin to a certain radian
  public double calculateMotorOutput(double radian) {
    double output = (m_pid.calculate(m_RightEncoder.getPosition().getValueAsDouble())
     + m_armFeed.calculate(radian, m_RightEncoder.getVelocity().getValueAsDouble()));
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

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
