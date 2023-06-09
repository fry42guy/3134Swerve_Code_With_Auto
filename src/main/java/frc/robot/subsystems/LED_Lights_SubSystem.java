// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED_Lights_SubSystem extends SubsystemBase {

  private final DoubleSolenoid m_LEDs = new DoubleSolenoid(Constants.LED_PCM_CAN_ID,PneumaticsModuleType.CTREPCM, 2, 4);


  /** Creates a new LED_Lights_SubSystem. */
  public LED_Lights_SubSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void LED_puple(){
m_LEDs.set(Value.kReverse);

  }

  public void LED_yellow(){
m_LEDs.set(Value.kForward);

  }
}
