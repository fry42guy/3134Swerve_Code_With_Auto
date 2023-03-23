// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake_ADV_BREAK_MODE extends CommandBase {
  private final ArmIntakeSubsystem m_IntakeSubsystem;
  /** Creates a new Intake_ADV_BREAK_MODE. */
  private PIDController m_Arm_LeftPIDController;
  private PIDController m_Arm_RightPIDController;
  
  
  public Intake_ADV_BREAK_MODE(ArmIntakeSubsystem m_IntakeSubsystem) {
    
   
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    m_Arm_LeftPIDController = new PIDController(.0003, 0.000001, 0.0);
    m_Arm_RightPIDController = new PIDController(.0003, 0.0000001, 0.0);
   // m_Arm_LeftPIDController.enableContinuousInput(-1, 1);
    m_Arm_LeftPIDController.setTolerance(1);
    m_Arm_RightPIDController.setTolerance(1);

    addRequirements(m_IntakeSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.Arm_Encoder_RESET();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double feedforwardl = 0.00;
    double speedL = m_Arm_LeftPIDController.calculate(m_IntakeSubsystem.getAbsoluteEncoderCountsLeft(), 0);
    speedL = (speedL > 0) ? speedL + feedforwardl : speedL - feedforwardl;
    speedL = (speedL > 1 ) ? 1.0 : speedL;
    speedL = (speedL < -1 ) ? -1 : speedL; 
    m_IntakeSubsystem.setSpeedLeft(speedL*.5);
    SmartDashboard.putNumber("Arm_Left output: ", speedL);

    double feedforwardR = 0.00;
    double speedR = m_Arm_RightPIDController.calculate(m_IntakeSubsystem.getAbsoluteEncoderCountsRight(), 0);
    speedR = (speedR > 0) ? speedR + feedforwardR : speedR - feedforwardR;
    speedR = (speedR > 1 ) ? 1.0 : speedR;
    speedR = (speedR < -1 ) ? -1 : speedR; 
    m_IntakeSubsystem.setSpeedRight(speedR*.5);
    SmartDashboard.putNumber("Arm_Right output: ", speedR);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;

  }

 
}