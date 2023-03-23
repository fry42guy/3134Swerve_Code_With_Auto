// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HorizontalSubsystem extends SubsystemBase {
  /** Creates a new horizontalSubsystem. */

private final TalonFX horizontalmotor;


  public HorizontalSubsystem() {
horizontalmotor = new TalonFX(Constants.m_Horizontal);

horizontalmotor.setSelectedSensorPosition(0);
horizontalmotor.configForwardSoftLimitThreshold(Constants.Horizontal_Limit_High);
horizontalmotor.configReverseSoftLimitThreshold(Constants.Horizontal_Limit_Low);
horizontalmotor.configForwardSoftLimitEnable(true);
horizontalmotor.configReverseSoftLimitEnable(true);
horizontalmotor.setNeutralMode(NeutralMode.Coast);



  }

  public void ZeroAxis(){
    horizontalmotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("horizontal", getAbsoluteEncoderCounts());
    // This method will be called once per scheduler run
  }




  public void setSpeed(double speed)
  {
    horizontalmotor.set(ControlMode.PercentOutput, speed);

  }

  public void stop()
  {
    horizontalmotor.set(ControlMode.PercentOutput,0);
  }
  public double getAbsoluteEncoderCounts()
  {
    return horizontalmotor.getSelectedSensorPosition();
  }

}
