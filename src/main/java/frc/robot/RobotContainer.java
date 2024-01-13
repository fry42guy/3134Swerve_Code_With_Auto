// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Auto_Balance_With_Nav_X;
import frc.robot.commands.IntakeFWD;
import frc.robot.commands.IntakeREV;
import frc.robot.commands.Intake_ADV_BREAK_MODE;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestMotorSetSpeed;
import frc.robot.subsystems.AirMod;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TestMotorSubSystem;

import java.util.List;
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

    private final TestMotorSubSystem motor1sub = new TestMotorSubSystem(Constants.Motor1.CANID);
    private final TestMotorSubSystem motor2sub = new TestMotorSubSystem(Constants.Motor2.CANID);
    private final TestMotorSubSystem motor3sub = new TestMotorSubSystem(Constants.Motor3.CANID);
    private final TestMotorSubSystem motor4sub = new TestMotorSubSystem(Constants.Motor4.CANID);

    private final ArmIntakeSubsystem m_ArmIntakeSubsystem = new ArmIntakeSubsystem();

    SendableChooser<List<PathPlannerTrajectory>> autoChooser_Path = new SendableChooser<>(); 
  //private String m_AutoSelected_Path;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    // Configure the trigger bindings
   CameraServer.startAutomaticCapture(0);
   
    configureBindings();
    m_ArmIntakeSubsystem.setDefaultCommand(new Intake_ADV_BREAK_MODE(m_ArmIntakeSubsystem));
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
 
          () -> m_Drive_Controller.getLeftY()*Math.abs(m_Drive_Controller.getLeftY())*.9, 
          () -> m_Drive_Controller.getLeftX()*Math.abs(m_Drive_Controller.getLeftX())*.9, 
          () -> m_Drive_Controller.getRightX()*Math.abs(m_Drive_Controller.getRightX())*-1, 
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


    new JoystickButton(m_Drive_Controller, Constants.Motor1.FWDButton) .whileTrue(new TestMotorSetSpeed(motor1sub,Constants.Motor1.FWDSpeed));
    new JoystickButton(m_Drive_Controller, Constants.Motor1.REVButton) .whileTrue(new TestMotorSetSpeed(motor1sub,Constants.Motor1.REVSpeed));

    new JoystickButton(m_Drive_Controller, Constants.Motor2.FWDButton) .whileTrue(new TestMotorSetSpeed(motor2sub,Constants.Motor2.FWDSpeed));
    new JoystickButton(m_Drive_Controller, Constants.Motor2.REVButton) .whileTrue(new TestMotorSetSpeed(motor2sub,Constants.Motor2.REVSpeed));

    new JoystickButton(m_Drive_Controller, Constants.Motor3.FWDButton) .whileTrue(new TestMotorSetSpeed(motor3sub,Constants.Motor3.FWDSpeed));
    new JoystickButton(m_Drive_Controller, Constants.Motor3.REVButton) .whileTrue(new TestMotorSetSpeed(motor3sub,Constants.Motor3.REVSpeed));
    
    new JoystickButton(m_Drive_Controller, Constants.Motor4.FWDButton) .whileTrue(new TestMotorSetSpeed(motor4sub,Constants.Motor4.FWDSpeed));
    new JoystickButton(m_Drive_Controller, Constants.Motor4.REVButton) .whileTrue(new TestMotorSetSpeed(motor4sub,Constants.Motor4.REVSpeed));

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





//


//






  
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
      new PIDConstants(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, 0),
      s_Swerve::setModuleStates,
      Constants.AutoConstants.eventMap,
      true,
      s_Swerve
  );

  return swerveAutoBuilder.fullAuto(trajs);
}

}
