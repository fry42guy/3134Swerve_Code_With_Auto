// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestMotorSubSystem;



public class TestMotorSetSpeed extends CommandBase {
  private final TestMotorSubSystem m_TestMotorSubSystem;
  private final Double m_Speed;
  /** Creates a new TestMotorSetSpeed. */
  public TestMotorSetSpeed(TestMotorSubSystem TestMotorSubSystem,Double Speed) {
this.m_TestMotorSubSystem = TestMotorSubSystem;
this.m_Speed = Speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_TestMotorSubSystem.SetSpeed(m_Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TestMotorSubSystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
