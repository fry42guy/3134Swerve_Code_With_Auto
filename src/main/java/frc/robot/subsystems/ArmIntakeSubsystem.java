// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmIntakeSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */

private final TalonFX Arm_Motor_Left;
private final TalonFX Arm_Motor_Right;


  public ArmIntakeSubsystem() {

Arm_Motor_Right = new TalonFX(Constants.Arm_Intake_Right_ID);
Arm_Motor_Left = new TalonFX(Constants.Arm_Intake_Left_ID);

Arm_Motor_Left.setSelectedSensorPosition(0);
Arm_Motor_Right.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm_Left", getAbsoluteEncoderCountsLeft());
    SmartDashboard.putNumber("Arm_Right", getAbsoluteEncoderCountsRight());
    // This method will be called once per scheduler run
  }




  public void setSpeed(double speed)
  {
    Arm_Motor_Left.set(ControlMode.PercentOutput, speed);
    Arm_Motor_Right.set(ControlMode.PercentOutput, speed*2);
  }

  public void setSpeedLeft(double speed){
    Arm_Motor_Left.set(ControlMode.PercentOutput, speed);

  }
  
  public void setSpeedRight(double speed){
    Arm_Motor_Right.set(ControlMode.PercentOutput, speed);

  }
  public void stop()
  {
    Arm_Motor_Left.set(ControlMode.PercentOutput,0);
    Arm_Motor_Right.set(ControlMode.PercentOutput,0);
  }
  public double getAbsoluteEncoderCountsLeft()
  {
    return Arm_Motor_Left.getSelectedSensorPosition();
  }
  public double getAbsoluteEncoderCountsRight()
  {
    return Arm_Motor_Right.getSelectedSensorPosition();
  }

  public void Arm_Encoder_RESET(){
    Arm_Motor_Left.setSelectedSensorPosition(0);
    Arm_Motor_Right.setSelectedSensorPosition(0);
  }

}
