// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualModeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.VerticalSubsystem;

public class VerticalManualMode_Up extends CommandBase {
  public final VerticalSubsystem m_VerticalSubsystem;
  /** Creates a new VerticalManualMode_Up. */
  public VerticalManualMode_Up(VerticalSubsystem m_VerticalSubsystem) {

    this.m_VerticalSubsystem = m_VerticalSubsystem;
    addRequirements(m_VerticalSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_VerticalSubsystem.setSpeed(Constants.Vertical_Motor_Speed*-1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_VerticalSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
