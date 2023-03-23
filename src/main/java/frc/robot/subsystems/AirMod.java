// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class AirMod extends SubsystemBase {
  /** Creates a new AirMod. */

  //private final PneumaticsControlModule m_Compressor = new PneumaticsControlModule(Constants.PCM_CAN_ID); 
  private final Compressor m_Compressor = new Compressor(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid m_Claw = new DoubleSolenoid(Constants.PCM_CAN_ID,PneumaticsModuleType.CTREPCM, 0, 2);


  public AirMod() {


  }
public void Start(){
m_Compressor.enableDigital();
  
}

public void ClawClose(){
m_Claw.set(Value.kForward);

}

public void ClawOpen(){
  m_Claw.set(Value.kReverse);
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
