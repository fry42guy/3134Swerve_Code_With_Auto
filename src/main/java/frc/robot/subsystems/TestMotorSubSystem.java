// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMotorSubSystem extends SubsystemBase {
  /** Creates a new TestMotorSubSystem. */


  private final TalonFX TestMotor;


  public TestMotorSubSystem(Integer CanID) {

TestMotor = new TalonFX(CanID);



  }

  public void SetSpeed(Double Speed){

    TestMotor.set(ControlMode.PercentOutput, Speed);


  }

  public void Stop (){
  TestMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
