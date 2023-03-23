// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualModeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HorizontalSubsystem;

public class HorizontalManualMode_Out extends CommandBase {
  /** Creates a new HorizontalManualMode. */
  public final HorizontalSubsystem m_HorizontalSubsystem;
  
  public HorizontalManualMode_Out(HorizontalSubsystem m_HorizontalSubsystem) {
    this.m_HorizontalSubsystem = m_HorizontalSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_HorizontalSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_HorizontalSubsystem.setSpeed(Constants.Horizontal_Motor_Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_HorizontalSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
