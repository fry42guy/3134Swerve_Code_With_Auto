// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.VerticalSubsystem;

public class PIDVerticalCommand_Auto extends CommandBase {
  /** Creates a new PIDVerticalCommand. */
  private PIDController m_VerticalPIDController;
  private final VerticalSubsystem m_VerticalSubsystem;
  private double setPoint;
  public PIDVerticalCommand_Auto(VerticalSubsystem m_VerticalSubsystem, double setPoint) {
    this.m_VerticalSubsystem = m_VerticalSubsystem;
    m_VerticalPIDController = new PIDController(.00004, 0.000001, 0.0);
   // m_VerticalPIDController.enableContinuousInput(-1, 1);
    m_VerticalPIDController.setTolerance(1000);
    this.setPoint = setPoint;
    addRequirements(m_VerticalSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_VerticalSubsystem.RampRate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_VerticalSubsystem.RampRate();
    double feedforward = -0.05;
    double speed = m_VerticalPIDController.calculate(m_VerticalSubsystem.getAbsoluteEncoderCounts(), setPoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    m_VerticalSubsystem.setSpeed(speed);
    SmartDashboard.putNumber("Vertical output: ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_VerticalSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_VerticalPIDController.atSetpoint());
    if(m_VerticalPIDController.atSetpoint())
          return true;
    return false;

  }

  public void setPoint(double setPoint)
  {
    this.setPoint = setPoint;
  }
}
