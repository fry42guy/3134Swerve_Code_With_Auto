// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

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

    SendableChooser<List<PathPlannerTrajectory>> autoChooser_Path = new SendableChooser<>(); 
  private String m_AutoSelected_Path;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    // Configure the trigger bindings
    CameraServer.startAutomaticCapture(0);
    setUpEventMap();
    configureBindings();

  }


  public void setUpAutos() {
    setUpEventMap();
    autoChooser_Path.setDefaultOption("Simple", PathPlanner.loadPathGroup("Simple", new PathConstraints(4.5, 3)));

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
          () -> m_Drive_Controller.getRightX(), 
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
