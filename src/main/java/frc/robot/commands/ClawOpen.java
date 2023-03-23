// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AirMod;

public class ClawOpen extends CommandBase {

  private final AirMod m_AirMod;
  private final boolean m_true;
  /** Creates a new ClawOpen. */
  public ClawOpen(AirMod m_AirMod,boolean Open) {
this.m_true = Open;
    this.m_AirMod = m_AirMod;

    addRequirements(m_AirMod);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
if (m_true){

  m_AirMod.ClawOpen();
}
else

m_AirMod.ClawClose();



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
