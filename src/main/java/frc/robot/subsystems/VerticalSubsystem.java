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


public class VerticalSubsystem extends SubsystemBase {
  /** Creates a new VerticalSubsystem. */

private final TalonFX Verticalmotor;


  public VerticalSubsystem() {


Verticalmotor = new TalonFX(Constants.m_Vertical);

Verticalmotor.setSelectedSensorPosition(0);
Verticalmotor.configReverseSoftLimitThreshold(Constants.Vertical_limit_Low);
Verticalmotor.configForwardSoftLimitThreshold(Constants.Vertical_Limit_High);
Verticalmotor.configForwardSoftLimitEnable(true);
Verticalmotor.configReverseSoftLimitEnable(true);
Verticalmotor.setNeutralMode(NeutralMode.Coast);
///////////////*************///////////////

  }

  public void ZeroAxis(){
    Verticalmotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Vertical", getAbsoluteEncoderCounts());
    // This method will be called once per scheduler run
  }




  public void setSpeed(double speed)
  {
    Verticalmotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop()
  {
    Verticalmotor.set(ControlMode.PercentOutput,0);
  }
  public double getAbsoluteEncoderCounts()
  {
    return Verticalmotor.getSelectedSensorPosition();
  }
public void encoder_reset(){
  Verticalmotor.setSelectedSensorPosition(0);
}

public void RampRate(){

  Verticalmotor.configMotionAcceleration(Constants.Vertical_Motio_Accel);
}
}
