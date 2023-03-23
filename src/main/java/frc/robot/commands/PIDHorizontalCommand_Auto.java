// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HorizontalSubsystem;

public class PIDHorizontalCommand_Auto extends CommandBase {
  /** Creates a new PIDHorizontalCommand. */
  private PIDController m_HorizontalPIDController;
  private final HorizontalSubsystem m_HorizontalSubsystem;
  private double setPoint;
  public PIDHorizontalCommand_Auto(HorizontalSubsystem m_HorizontalSubsystem, double setPoint) {
    this.m_HorizontalSubsystem = m_HorizontalSubsystem;
    m_HorizontalPIDController = new PIDController(.004, 0.0000001, 0.0);
    
   // m_HorizontalPIDController.enableContinuousInput(-1, 1);
    m_HorizontalPIDController.setTolerance(1000);
    this.setPoint = setPoint;
    addRequirements(m_HorizontalSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double feedforward = -0.05;
    double speed = m_HorizontalPIDController.calculate(m_HorizontalSubsystem.getAbsoluteEncoderCounts(), setPoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    speed = (speed > 1 ) ? 1.0 : speed;
    speed = (speed < -1 ) ? -1 : speed; 
    m_HorizontalSubsystem.setSpeed(speed* Constants.Horizontal_PID_Speed);
    SmartDashboard.putNumber("Horizontal output: ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_HorizontalSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_HorizontalPIDController.atSetpoint())
          return true;
    return false;

  }

  public void setPoint(double setPoint)
  {
    this.setPoint = setPoint;
  }
}
