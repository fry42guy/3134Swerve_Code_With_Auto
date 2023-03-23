// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmIntakeSubsystem;

public class AutoShoot extends CommandBase {
  private final ArmIntakeSubsystem m_IntakeSubsystem;

  private final Timer m_time = new Timer();
private final double m_seconds;
private final double m_speed;

  /** Creates a new IntakeFWD. */
  public AutoShoot(ArmIntakeSubsystem m_IntakeSubsystem,double speed, double Time) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.m_IntakeSubsystem = m_IntakeSubsystem;
  this.m_seconds = Time;
  this.m_speed = speed;
  
  addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_IntakeSubsystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_time.get()> m_seconds){
      return true;
    }
    else{
    return false;
  }
}}
