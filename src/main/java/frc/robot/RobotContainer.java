// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeFWD;
import frc.robot.commands.IntakeREV;
import frc.robot.commands.Intake_ADV_BREAK_MODE;
import frc.robot.commands.PIDHorizontalCommand;
import frc.robot.commands.PIDVerticalCommand;
import frc.robot.commands.PIDWristCommand;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ManualModeCommands.HorizontalManualMode_In;
import frc.robot.commands.ManualModeCommands.HorizontalManualMode_Out;
import frc.robot.commands.ManualModeCommands.VerticalManualMode_Down;
import frc.robot.commands.ManualModeCommands.VerticalManualMode_Up;
import frc.robot.commands.ManualModeCommands.WristManualMode_Down;
import frc.robot.commands.ManualModeCommands.WristManualMode_Up;
import frc.robot.subsystems.AirMod;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.HorizontalSubsystem;
import frc.robot.subsystems.LED_Lights_SubSystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VerticalSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static SwerveAutoBuilder swerveAutoBuilder;
  public static Swerve s_Swerve = new Swerve();
    public static Limelight m_Limelight = new Limelight();
    private final XboxController m_Drive_Controller = new XboxController(0);

    private final XboxController m_Operator_Controller = new XboxController(1);

    private final AirMod M_PCM = new AirMod();

    private final LED_Lights_SubSystem m_LED = new LED_Lights_SubSystem();
  
    private final WristSubsystem m_Wrist = new WristSubsystem();
  
    private final VerticalSubsystem m_Vertical = new VerticalSubsystem();
  
    private final HorizontalSubsystem m_Horizontal = new HorizontalSubsystem();
  
    private final ArmIntakeSubsystem m_ArmIntakeSubsystem = new ArmIntakeSubsystem();

    SendableChooser<List<PathPlannerTrajectory>> autoChooser_Path = new SendableChooser<>(); 
  //private String m_AutoSelected_Path;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    // Configure the trigger bindings
    CameraServer.startAutomaticCapture(0);
    setUpEventMap();
    configureBindings();
    m_ArmIntakeSubsystem.setDefaultCommand(new Intake_ADV_BREAK_MODE(m_ArmIntakeSubsystem));
  }


  public void setUpAutos() {
    setUpEventMap();
    autoChooser_Path.setDefaultOption("Simple", PathPlanner.loadPathGroup("Simple", new PathConstraints(2, 2)));
    autoChooser_Path.addOption("Not_Simple", PathPlanner.loadPathGroup("Not_Simple", new PathConstraints(2, 2)));
    autoChooser_Path.addOption("Score_Backup", PathPlanner.loadPathGroup("Score_Backup", new PathConstraints(2, 2)));
    autoChooser_Path.addOption("Score_Charge", PathPlanner.loadPathGroup("Score_Charge", new PathConstraints(2, 2)));
    
    autoTab.add(autoChooser_Path);

  }

  public void setUpEventMap() { // put actions here
    Constants.AutoConstants.eventMap.clear();

//     Constants.AutoConstants.eventMap.put("ScoreMid", new SequentialCommandGroup( // runs a group sequentialy between the ( ) 
//     new ParallelCommandGroup( 
//   ( new PIDVerticalCommand_Auto(m_Vertical, Constants.Cone_Cube_MID_Vert+ Constants.Vertical_PID_Tolerance_Offset)),
//   (new PIDHorizontalCommand_Auto(m_Horizontal, Constants.Cone_Cube_MID_Hori+ Constants.Horizontal_PID_Tolerance_Offset)),
//   ( new PIDWristCommand_Auto(m_Wrist, Constants.Cone_Cube_MID_Wrist+Constants.Wrist_PID_Tolerance_Offset))),

//   new ParallelCommandGroup(
//   new AutoShoot(m_ArmIntakeSubsystem, .3, 1)))
// );


// Constants.AutoConstants.eventMap.put("Stoe",   new ParallelCommandGroup(
      
    
//       (new PIDVerticalCommand_Auto(m_Vertical, Constants.Store_Stoe_Vert + Constants.Vertical_PID_Tolerance_Offset)),
//           (new PIDHorizontalCommand_Auto(m_Horizontal, Constants.Store_Stoe_Hori + Constants.Horizontal_PID_Tolerance_Offset)),
//           (new PIDWristCommand_Auto(m_Wrist, Constants.Store_Stoe_Wrist+Constants.Wrist_PID_Tolerance_Offset))
//        ));
    
    
//        Constants.AutoConstants.eventMap.put("FloorPickup",   new ParallelCommandGroup(
      
    
//         (new PIDVerticalCommand_Auto(m_Vertical, Constants.Floor_Cube_Cone_Vert + Constants.Vertical_PID_Tolerance_Offset)),
//             (new PIDHorizontalCommand_Auto(m_Horizontal, Constants.Floor_Cube_Cone_Hori + Constants.Horizontal_PID_Tolerance_Offset)),
//             (new PIDWristCommand_Auto(m_Wrist, Constants.Floor_Cube_Cone_Wrist+Constants.Wrist_PID_Tolerance_Offset))
//          ));

//          Constants.AutoConstants.eventMap.put("IntakeCone", new ParallelCommandGroup(new AutoShoot(m_ArmIntakeSubsystem, .3, 2)));



  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */



   public void resetToAbsolute() {
    s_Swerve.resetModulesToAbsolute();
}
public void scheduleDefaultTeleop() {
  s_Swerve.setDefaultCommand(
      new TeleopSwerve(
          s_Swerve, 
          // () -> -modifyAxis(m_Drive_Controller.getLeftY()*.8),// * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          // () -> -modifyAxis(m_Drive_Controller.getLeftX()*.8), //* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          // () -> -modifyAxis(m_Drive_Controller.getRightX()*.7),// * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
 
          () -> -m_Drive_Controller.getLeftY()*Math.abs(m_Drive_Controller.getLeftY())*.8, 
          () -> -m_Drive_Controller.getLeftX()*Math.abs(m_Drive_Controller.getLeftX())*.8, 
          () -> m_Drive_Controller.getRightX()*Math.abs(m_Drive_Controller.getRightX()), 
          () -> false //() -> robotCentric.getAsBoolean() //always field centric
      )
  );

  // m_Arm.setDefaultCommand(armControls);
  // m_Elevator.setDefaultCommand(elevatorControls);
  // m_Intake.setDefaultCommand(new RunIntake(m_Intake, driver, m_LEDs));


  // Configure the button bindings
  configureBindings();
  m_Limelight.initializeLimeLight();
}

public void cancelDefaultTeleop() {
  s_Swerve.getDefaultCommand().cancel();
 // m_Arm.getDefaultCommand().cancel();
 // m_Elevator.getDefaultCommand().cancel();
//  m_Intake.getDefaultCommand().cancel();
}

public void zeroGyro() {
  s_Swerve.zeroGyro();
}
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   new JoystickButton(m_Drive_Controller, XboxController.Button.kRightStick.value) .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  
   new Trigger(() ->
   {if(m_Operator_Controller.getLeftTriggerAxis() >0)
    return true;
    else
    {
      return false;
    }
   
   }
   ).onTrue(new InstantCommand(
   ()-> M_PCM.ClawClose()));

new JoystickButton(m_Operator_Controller, XboxController.Button.kLeftBumper.value)
.onTrue(new InstantCommand(
()-> M_PCM.ClawOpen()
));

new JoystickButton(m_Operator_Controller, XboxController.Button.kX.value)
.onTrue(new InstantCommand(
()-> m_LED.LED_puple()
));

new JoystickButton(m_Operator_Controller, XboxController.Button.kY.value)
.onTrue(new InstantCommand(
()-> m_LED.LED_yellow()
));

  
new Trigger(()->
     
{
  if(m_Drive_Controller.getRightTriggerAxis() > 0)
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.Store_Stoe_Vert + Constants.Vertical_PID_Tolerance_Offset))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.Store_Stoe_Hori + Constants.Horizontal_PID_Tolerance_Offset))
.onTrue(new PIDWristCommand(m_Wrist, Constants.Store_Stoe_Wrist+Constants.Wrist_PID_Tolerance_Offset));

new Trigger(()->

{
  if(m_Drive_Controller.getLeftBumper())
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.Floor_Cube_Cone_Vert+ Constants.Vertical_PID_Tolerance_Offset))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.Floor_Cube_Cone_Hori+ Constants.Horizontal_PID_Tolerance_Offset))
.onTrue(new PIDWristCommand(m_Wrist, Constants.Floor_Cube_Cone_Wrist+Constants.Wrist_PID_Tolerance_Offset));

new Trigger(()->

{
  if(m_Drive_Controller.getBButton())
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.Cube_High_Vert+ Constants.Vertical_PID_Tolerance_Offset))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.Cube_high_Hori+ Constants.Horizontal_PID_Tolerance_Offset))
.onTrue(new PIDWristCommand(m_Wrist, Constants.Cube_High_Wrist+Constants.Wrist_PID_Tolerance_Offset));

new Trigger(()->

{  
  if(m_Drive_Controller.getLeftTriggerAxis() > 0)
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.Cone_Cube_MID_Vert+ Constants.Vertical_PID_Tolerance_Offset))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.Cone_Cube_MID_Hori+ Constants.Horizontal_PID_Tolerance_Offset))
.onTrue(new PIDWristCommand(m_Wrist, Constants.Cone_Cube_MID_Wrist+Constants.Wrist_PID_Tolerance_Offset));


new Trigger(()->

{
  if(m_Drive_Controller.getXButton())
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.Cone_Cube_High_Vert+ Constants.Vertical_PID_Tolerance_Offset))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.Cone_Cube_High_Hori+ Constants.Horizontal_PID_Tolerance_Offset))
.onTrue(new PIDWristCommand(m_Wrist, Constants.Cone_Cube_High_Wrist+Constants.Wrist_PID_Tolerance_Offset));



new Trigger(()->

{
  if(m_Drive_Controller.getRightBumper())
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.Cone_Cube_Travel_Vert+ Constants.Vertical_PID_Tolerance_Offset))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.Cone_Cube_Travel_Hori+ Constants.Horizontal_PID_Tolerance_Offset))
.onTrue(new PIDWristCommand(m_Wrist, Constants.Cone_Cube_Travel_Wrist+Constants.Wrist_PID_Tolerance_Offset));

new Trigger(()->

{
  if(m_Drive_Controller.getYButton())
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.Cone_Cube_Player_Station_Vert+ Constants.Vertical_PID_Tolerance_Offset))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.Cone_Cube_Player_Station_Hori+ Constants.Horizontal_PID_Tolerance_Offset))
.onTrue(new PIDWristCommand(m_Wrist, Constants.Cone_Cube_Player_Station_Wrist+Constants.Wrist_PID_Tolerance_Offset));

new Trigger(()->

{
  if(m_Drive_Controller.getAButton())
    return true;
  else
    return false;

})
.onTrue(new PIDVerticalCommand(m_Vertical, Constants.BB_Virt))
.onTrue(new PIDHorizontalCommand(m_Horizontal, Constants.BB_Hori))
.onTrue(new PIDWristCommand(m_Wrist, Constants.BB_Wrist));

  
new Trigger(() ->
{if(m_Operator_Controller.getRightTriggerAxis()>0)
 return true;
 else
 {
   return false;
 }

}
).whileTrue(new IntakeFWD(m_ArmIntakeSubsystem));



new Trigger(() ->
{if(m_Operator_Controller.getRightBumper() &! m_Operator_Controller.getLeftStickButton())
 return true;
 else
 {
   return false;
 }

}
).whileTrue(new IntakeREV(m_ArmIntakeSubsystem));


new Trigger(() -> 

{if (m_Operator_Controller.getLeftStickButton()&& m_Operator_Controller.getAButton())

return true;
else
return false;
}
).whileTrue(new VerticalManualMode_Down (m_Vertical));


new Trigger(() ->
{if (m_Operator_Controller.getLeftStickButton()&& m_Operator_Controller.getBButton())

 return true;
 else
  return false;
 }
 ).whileTrue(new VerticalManualMode_Up (m_Vertical));


//
new Trigger(() -> 

{if (m_Operator_Controller.getLeftStickButton()&& m_Operator_Controller.getRightBumper())

return true;
else
return false;
}
).whileTrue(new HorizontalManualMode_Out (m_Horizontal));


new Trigger(() ->
{if (m_Operator_Controller.getLeftStickButton()&& m_Operator_Controller.getLeftBumper())

 return true;
 else
  return false;
 }
 ).whileTrue(new HorizontalManualMode_In (m_Horizontal));
  
//

new Trigger(() -> 

{if (m_Operator_Controller.getLeftStickButton()&& m_Operator_Controller.getYButton())

return true;
else
return false;
}
).whileTrue(new WristManualMode_Up (m_Wrist));


new Trigger(() ->
{if (m_Operator_Controller.getLeftStickButton()&& m_Operator_Controller.getXButton())

 return true;
 else
  return false;
 }
 ).whileTrue(new WristManualMode_Down (m_Wrist));







  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {



 

    // An example command will be run in autonomous
   return buildAuto(autoChooser_Path.getSelected());
}


public static Command buildAuto(List<PathPlannerTrajectory> trajs) {
  //s_Swerve.resetOdometry(trajs.get(0).getInitialHolonomicPose());
  swerveAutoBuilder = new SwerveAutoBuilder(
      s_Swerve::getPose,
      s_Swerve::resetOdometry,
      Constants.Swerve.swerveKinematics,
      new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
      new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
      s_Swerve::setModuleStates,
      Constants.AutoConstants.eventMap,
      true,
      s_Swerve
  );

  return swerveAutoBuilder.fullAuto(trajs);
}

}
