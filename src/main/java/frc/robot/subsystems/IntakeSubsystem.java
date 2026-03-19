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
  private final SparkFlex m_motorRightIntake;
  private final SparkFlex m_motorRightSpin;
  private final XboxController m_driveController;
  private final PIDController m_pid;

  private final SparkFlexConfig m_motorConfig = new SparkFlexConfig();

  private final CANcoder m_RightEncoder;
  private final ArmFeedforward m_armFeed;


  public Trigger runIntakeTrigger = new Trigger(() -> {return false;});
  public Trigger reverseIntakeTrigger = new Trigger(() -> {return false;});


 
  public IntakeSubsystem() {
    m_motorRightIntake = new SparkFlex(Constants.IntakeConstants.k_RightIntakeId, SparkLowLevel.MotorType.kBrushless);
    m_motorRightSpin = new SparkFlex(Constants.IntakeConstants.k_RightSpinId, SparkLowLevel.MotorType.kBrushless);
    m_driveController = new XboxController(Constants.OperatorConstants.kHelperControllerPort);
    m_pid = new PIDController(Constants.IntakeConstants.k_Kp, 0, Constants.IntakeConstants.k_Kd);
    m_RightEncoder = new CANcoder(Constants.IntakeConstants.k_RightEncoderId);
    m_armFeed = new ArmFeedforward(Constants.IntakeConstants.k_Ks, Constants.IntakeConstants.k_Kg, Constants.IntakeConstants.k_Kv);
  }

  public Command runCommand() {
    return run(
        () -> {
  
          //Right Motor Code
          //Spin Motor Code
          runIntakeTrigger.whileTrue(runIntakeCommand().onlyIf(() -> {return !reverseIntakeTrigger.getAsBoolean();}));
            
            
    

          reverseIntakeTrigger.whileTrue(reverseIntakeCommand().onlyIf(() -> {return runIntakeTrigger.getAsBoolean();}));

          

          //Have it so if either triggers are pressed, then run the intake being brought down
          //
                        //Reverse Motor Code
            /* 
            if (reverseIntakeTrigger.getAsBoolean()){
                m_motorRightIntake.set(-Constants.IntakeConstants.k_IntakePower);
              } else { 
                 m_motorRightIntake.set(Constants.IntakeConstants.k_IntakePower);
              }
            /*if (m_encoder.getPosition().getValueAsDouble() >= 0.25) {
              m_motorRightSpin.set(0);
            }

          
          } else {
            m_motorRightSpin.set(calculateMotorOutput(0));
                      }
            */
          /* 
          if (m_driveController.getAButton() && !m_driveController.getBButton()) {
            m_motorLeftSpin.set(Constants.IntakeConstants.k_SpinPower);
          } else {
            m_motorLeftSpin.set(-Constants.IntakeConstants.k_SpinPower);
          }
          */
          //Left Motor Code
         
        


        });
  }

  //Creates the output needed for the motor spin to a certain radian
  public double calculateMotorOutput(double radian) {
    double output = 0;
      output = (m_pid.calculate(m_RightEncoder.getPosition().getValueAsDouble())
     + m_armFeed.calculate(radian, m_RightEncoder.getVelocity().getValueAsDouble()));
    return output;
  }

  public Command reverseIntakeCommand() {
    return Commands.runOnce(() -> {
      m_motorRightSpin.set(calculateMotorOutput((Constants.IntakeConstants.k_TargetAngle * Math.PI)/180));
      m_motorRightIntake.set(-Constants.IntakeConstants.k_IntakePower);
    });
  }

  public Command runIntakeCommand() {
    return Commands.runOnce(() -> {
      m_motorRightSpin.set(calculateMotorOutput((Constants.IntakeConstants.k_TargetAngle * Math.PI)/180));
      m_motorRightIntake.set(Constants.IntakeConstants.k_IntakePower);
    });
  }

    //Insert getAngularVelocity method
  

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
