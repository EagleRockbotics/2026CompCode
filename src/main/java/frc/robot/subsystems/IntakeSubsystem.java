// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants;
import swervelib.encoders.CanAndMagSwerve;

import com.ctre.phoenix6.hardware.CANcoder;



public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_motorFrontIntake;
  private final SparkFlex m_motorFrontSpin;
  private final SparkFlex m_motorBackIntake;
  private final SparkFlex m_motorBackSpin;
  private final XboxController m_driveController;
  private final PIDController m_pid;

  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();

  private final CANcoder m_encoder;
  private final ArmFeedforward m_armFeed;


 
  public IntakeSubsystem() {
    m_motorFrontIntake = new SparkFlex(Constants.IntakeConstants.k_FrontIntakeId, null);
    m_motorFrontSpin = new SparkFlex(Constants.IntakeConstants.k_FrontSpinId, null);
    m_motorBackIntake = new SparkFlex(Constants.IntakeConstants.k_BackIntakeId, null);
    m_motorBackSpin = new SparkFlex(Constants.IntakeConstants.k_BackSpinId,null );
    m_driveController = new XboxController(0);
    m_pid = new PIDController(Constants.IntakeConstants.k_Kp, 0, Constants.IntakeConstants.k_Kd);
    m_encoder = new CANcoder(0);
    m_armFeed = new ArmFeedforward(Constants.IntakeConstants.k_Ks, Constants.IntakeConstants.k_Kg, Constants.IntakeConstants.k_Kv);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */ 
  public Command runCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          //Spin Motor Code
          if (m_driveController.getBButton() && !m_driveController.getAButton()){
            m_motorFrontSpin.set(calculateMotorOutput((Constants.IntakeConstants.k_TargetAngle * Math.PI)/180));
          
            /*if (m_encoder.getPosition().getValueAsDouble() >= 0.25) {
              m_motorFrontSpin.set(0);
            }*/
          } else {
            m_motorFrontSpin.set(calculateMotorOutput(0));
                      }
          /* 
          if (m_driveController.getAButton() && !m_driveController.getBButton()) {
            m_motorBackSpin.set(Constants.IntakeConstants.k_SpinPower);
          } else {
            m_motorBackSpin.set(-Constants.IntakeConstants.k_SpinPower);
          }
          */
          if (m_driveController.getAButton() && !m_driveController.getBButton()){
            m_motorBackSpin.set(calculateMotorOutput((Constants.IntakeConstants.k_TargetAngle * Math.PI)/ 180));
          } else {
            m_motorBackSpin.set(calculateMotorOutput(0));
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


  public double calculateMotorOutput(double radian) {
    double output = (m_pid.calculate(m_encoder.getPosition().getValueAsDouble())
     + m_armFeed.calculate(radian, m_encoder.getVelocity().getValueAsDouble()));
    return output;
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
