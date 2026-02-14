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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_motorFrontIntake;
  private final SparkFlex m_motorFrontSpin;
  private final SparkFlex m_motorBackIntake;
  private final SparkFlex m_motorBackSpin;
  private final XboxController m_driveController;

  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();


 
  public IntakeSubsystem() {
    m_motorFrontIntake = new SparkFlex(Constants.IntakeConstants.k_FrontIntakeId, null);
    m_motorFrontSpin = new SparkFlex(Constants.IntakeConstants.k_FrontSpinId, null);
    m_motorBackIntake = new SparkFlex(Constants.IntakeConstants.k_BackIntakeId, null);
    m_motorBackSpin = new SparkFlex(Constants.IntakeConstants.k_BackSpinId, );
    m_driveController = new XboxController(0);
  
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
          if (m_driveController.getBButton() && !m_driveController.getBButton()){
            m_motorFrontSpin.set(1);
          } else {
            m_motorFrontSpin.set(0);
          }

          if (m_driveController.getAButton() && !m_driveController.getBButton()) {
            m_motorBackSpin.set(1);
          } else {
            m_motorBackSpin.set(0);
          }
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
