// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmIntakeSubsystem;

public class IntakeFWD extends CommandBase {
  private final ArmIntakeSubsystem m_IntakeSubsystem;
  /** Creates a new IntakeFWD. */
  public IntakeFWD(ArmIntakeSubsystem m_IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.m_IntakeSubsystem = m_IntakeSubsystem;
  addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_IntakeSubsystem.setSpeed(Constants.Arm_intake_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
