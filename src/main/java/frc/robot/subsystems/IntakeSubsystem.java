// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;



public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_motorFrontIntake;
  private final SparkFlex m_motorFrontSpin;
  private final SparkFlex m_motorBackIntake;
  private final SparkFlex m_motorBackSpin;
  private final XboxController m_driveController;
  private final PIDController m_pid;

  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();

  private final CANcoder m_frontEncoder;
  private final CANcoder m_backEncoder;
  private final ArmFeedforward m_armFeed;

  private final int FRONT = 1;
  private final int BACK = 2;


 
  public IntakeSubsystem() {
    m_motorFrontIntake = new SparkFlex(Constants.IntakeConstants.k_FrontIntakeId, SparkLowLevel.MotorType.kBrushless);
    m_motorFrontSpin = new SparkFlex(Constants.IntakeConstants.k_FrontSpinId, SparkLowLevel.MotorType.kBrushless);
    m_motorBackIntake = new SparkFlex(Constants.IntakeConstants.k_BackIntakeId, SparkLowLevel.MotorType.kBrushless);
    m_motorBackSpin = new SparkFlex(Constants.IntakeConstants.k_BackSpinId,SparkLowLevel.MotorType.kBrushless);
    m_driveController = new XboxController(Constants.OperatorConstants.kHelperControllerPort);
    m_pid = new PIDController(Constants.IntakeConstants.k_Kp, 0, Constants.IntakeConstants.k_Kd);
    m_frontEncoder = new CANcoder(Constants.IntakeConstants.k_frontEncoderId);
    m_backEncoder = new CANcoder(Constants.IntakeConstants.k_BackEncoderId);
    m_armFeed = new ArmFeedforward(Constants.IntakeConstants.k_Ks, Constants.IntakeConstants.k_Kg, Constants.IntakeConstants.k_Kv);
  }

  public Command runCommand() {
    return run(
        () -> {
          //Spin Motor Code
          if (m_driveController.getBButton() &&
           m_backEncoder.getPosition().getValueAsDouble() < Constants.IntakeConstants.k_EncoderThreshold){
            m_motorFrontSpin.set(calculateMotorOutput((Constants.IntakeConstants.k_TargetAngle * Math.PI)/180, FRONT));
          
            /*if (m_encoder.getPosition().getValueAsDouble() >= 0.25) {
              m_motorFrontSpin.set(0);
            }*/
          } else {
            m_motorFrontSpin.set(calculateMotorOutput(0, FRONT));
                      }
          /* 
          if (m_driveController.getAButton() && !m_driveController.getBButton()) {
            m_motorBackSpin.set(Constants.IntakeConstants.k_SpinPower);
          } else {
            m_motorBackSpin.set(-Constants.IntakeConstants.k_SpinPower);
          }
          */
          if (m_driveController.getAButton() &&
           m_frontEncoder.getPosition().getValueAsDouble() < Constants.IntakeConstants.k_EncoderThreshold){
            m_motorBackSpin.set(calculateMotorOutput((Constants.IntakeConstants.k_TargetAngle * Math.PI)/ 180, BACK));
          } else {
            m_motorBackSpin.set(calculateMotorOutput(0, BACK));
          }
          //Intake Code
          if (m_driveController.getRightBumperButton()) {
            m_motorFrontIntake.set(Constants.IntakeConstants.k_IntakePower);
          }
          if (m_driveController.getLeftBumperButton()) {
            m_motorBackIntake.set(Constants.IntakeConstants.k_IntakePower);
          }

        });
  }

  //Creates the output needed for the motor spin to a certain radian
  public double calculateMotorOutput(double radian, int encoder) {
    double output = 0;
    if (encoder == FRONT){
      output = (m_pid.calculate(m_frontEncoder.getPosition().getValueAsDouble())
     + m_armFeed.calculate(radian, m_frontEncoder.getVelocity().getValueAsDouble()));
    } else if(encoder == BACK){
      output = (m_pid.calculate(m_backEncoder.getPosition().getValueAsDouble())
     + m_armFeed.calculate(radian, m_backEncoder.getVelocity().getValueAsDouble()));
    }
    return output;
  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
