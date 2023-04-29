// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.math.BigDecimal;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.TeleopSwerve;




public class Auto_Balance_With_Nav_X extends CommandBase {


  private Swerve s_Swerve; 
  private double m_Pitch;
  private double m_Roll;
  private double m_Pitch_Speed;
  private double m_Roll_Speed;
private double x_Speed;
private double y_Speed;
 private boolean m_ontop;

  private PIDController m_PitchPIDController;
  private PIDController m_RollPidController;

  private Timer m_Timer;
  private Timer m_Timer2;

  private double Split_Time;




  /** Creates a new Auto_Balance_With_Nav_X. */
  public Auto_Balance_With_Nav_X(Swerve s_Swerve) {
this.s_Swerve = s_Swerve;

    m_PitchPIDController = new PIDController(.08, 0, 0); //.07 works
    m_RollPidController = new PIDController(.08, 0, 0);

m_Timer = new Timer();
m_Timer2 = new Timer();



m_PitchPIDController.setTolerance(2);
m_RollPidController.setTolerance(2);



    

    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.restart();
    m_ontop = false;
    m_Timer2.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Pitch = s_Swerve.getPitch();
    m_Roll = s_Swerve.getRoll();

    if(m_Timer2.get()>1){
      m_ontop = true;
    }

// SmartDashboard.putNumber("swerve get pitch", s_Swerve.getPitch());

System.out.println("running");
    SmartDashboard.putNumber("Nav_X Pitch", m_Pitch);
    SmartDashboard.putNumber("Nav_X Roll", m_Roll);

y_Speed = getPitchSpeed();

x_Speed = getRollSpeed();


Split_Time = Math.round( m_Timer.get()*3);
if (m_ontop==true){

if(Split_Time % 2 == 0){
  s_Swerve.drive(
      new Translation2d(-.05*x_Speed,-.05*y_Speed),0,false,true);
}
else{
    s_Swerve.drive(
      new Translation2d(-x_Speed,-y_Speed),0,false,true);
}
}

if (m_ontop ==false){
  s_Swerve.drive( new Translation2d(-x_Speed,-y_Speed),0,false,true);}

}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //m_Swerve.drive(new ChassisSpeeds(0, 0, .06));
   // Timer.delay(.05);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


public double getPitchSpeed(){

  m_Pitch_Speed = m_PitchPIDController.calculate(m_Pitch, 0);
  m_Pitch_Speed = (m_Pitch_Speed > 1) ? 1.0 : m_Pitch_Speed;
  m_Pitch_Speed = (m_Pitch_Speed < -1 ) ? -1 : m_Pitch_Speed;

  if(Math.abs(m_Pitch)<5){

    m_Pitch_Speed = m_Pitch_Speed*.08;
  }


  
if(m_PitchPIDController.atSetpoint()){
  m_Pitch_Speed = 0;
}
  return m_Pitch_Speed;
}
public double getRollSpeed(){

  m_Roll_Speed = m_RollPidController.calculate(m_Roll, 0);
m_Roll_Speed = (m_Roll_Speed >1) ? 1.0 : m_Roll_Speed;
m_Roll_Speed = (m_Roll_Speed < -1 ) ? -1 : m_Roll_Speed;

if (Math.abs(m_Roll)<5){

  m_Roll_Speed = m_Roll_Speed*.08;
}


if(m_RollPidController.atSetpoint()){
  m_Roll_Speed=0;
}

  return m_Roll_Speed;
}





}
