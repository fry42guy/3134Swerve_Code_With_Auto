// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */

private final TalonFX wristmotor;


  public WristSubsystem() {
wristmotor = new TalonFX(Constants.m_Wrist);

wristmotor.setSelectedSensorPosition(0);
wristmotor.configForwardSoftLimitThreshold(Constants.Wrist_Limit_High);
wristmotor.configReverseSoftLimitThreshold(Constants.Wrist_Limit_Low);
wristmotor.configForwardSoftLimitEnable(true);
wristmotor.configReverseSoftLimitEnable(true);


//wristmotor.configForwardSoftLimitThreshold(Constants.Wrist_Limit_High);
//wristmotor.configReverseSoftLimitThreshold(Constants.Wrist_Limit_Low);


  }

  public void ZeroAxis(){
    wristmotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Wrist", getAbsoluteEncoderCounts());
    // This method will be called once per scheduler run
  }




  public void setSpeed(double speed)
  {
    wristmotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop()
  {
    wristmotor.set(ControlMode.PercentOutput,0);
  }
  public double getAbsoluteEncoderCounts()
  {
    return wristmotor.getSelectedSensorPosition();
  }

}
