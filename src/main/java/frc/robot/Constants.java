// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static final double stickDeadband = 0.05;
  public static final int LED_PCM_CAN_ID = 20;

  public static final int PCM_CAN_ID = 1 ;
  public static final int Arm_Intake_Left_ID = 51;
  public static final int Arm_Intake_Right_ID = 50;
  public static final double Arm_intake_speed = .45;


  public static final int m_Wrist = 44; // ArmMotor Falcon 500 CAN ID ###
  public static final double Wrist_Motor_Speed = .2; 
  public static final double Wrist_Limit_High = 123000;
  public static final double Wrist_Limit_Low = 0;

  public static final int m_Vertical = 45; // ArmMotor Falcon 500 CAN ID ###
  public static final double Vertical_Motor_Speed = .4; 
  public static final double Vertical_Motio_Accel = 100; //units per 100ms - 10- 100ms/sec 4000units/rev 
  public static final double Vertical_Limit_High = 0;
  public static final double Vertical_limit_Low = -190000;
  
  public static final double Vertical_High_Setpoint = -180000;
  public static final double Vertical_Low_Setpoint = -4000;

  public static final double Vertical_PID_Tolerance_Offset = 0;
  public static final double Horizontal_PID_Tolerance_Offset = 0;
  public static final double Wrist_PID_Tolerance_Offset = 0;


  public static final double Wrist_High_Setpoint = 105000;
  public static final double Wrist_cube_Highth = 100000;
  public static final double Wrist_Low_Setpoint = 10000;
  public static final double Wrist_PID_Speed = .3;


  public static final int m_Horizontal = 46; // ArmMotor Falcon 500 CAN ID ###
  public static final double Horizontal_Motor_Speed = .2; 
  public static final double Horizontal_PID_Speed = .4;
  public static final double Horizontal_Limit_High = 105000 ;
  public static final double Horizontal_Limit_Low = 0;
  
  // Start/Stow (RT) new arm floor cube
public static final double Store_Stoe_Vert = -4000;
public static final double Store_Stoe_Wrist = 98000;
public static final double Store_Stoe_Hori = 100;

//BB(A) new arm
public static final double BB_Virt = -4000;
public static final double BB_Wrist =36300;
public static final double BB_Hori = 100;  


//Floor Pick up Cube/Cone(LB) new arm cone
public static final double Floor_Cube_Cone_Vert = -4000;
public static final double Floor_Cube_Cone_Wrist = 89368-2000;
public static final double Floor_Cube_Cone_Hori = 100;

//Score Cone/Cube MID (LT) new arm
public static final double Cone_Cube_MID_Vert = -55000;
public static final double Cone_Cube_MID_Wrist = 50095;
public static final double Cone_Cube_MID_Hori = 80082;


//Score Cone/Cube High (x) new arm cone shoot high
public static final double Cone_Cube_High_Vert = -55000;//-168192;
public static final double Cone_Cube_High_Wrist = 4500;//67000-20000-3000-3000;
public static final double Cone_Cube_High_Hori = 600;//103018;

//Travel w/Cone Cube(RB) new arm
public static final double Cone_Cube_Travel_Vert = -4000;
public static final double Cone_Cube_Travel_Wrist = 500;
public static final double Cone_Cube_Travel_Hori = 100;

//Pickup Player Station (B)cube high
public static final double Cube_High_Vert = -177192;
public static final double Cube_High_Wrist = 35000;
public static final double Cube_high_Hori = 80000;

//Pickup Player Station (Y) New arm cone cube
public static final double Cone_Cube_Player_Station_Vert = -177192;
public static final double Cone_Cube_Player_Station_Wrist = 65000;
public static final double Cone_Cube_Player_Station_Hori = 00;


  public static class OperatorConstants {
    
  }





  public static final class Swerve {
    
   // public static final int pigeonID = 1;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW- // I belive Navx is true
    
    public static final String CANivore = "CANt_open_file";// name of the canivore

    public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.58; // 20.5 in -> meters
    public static final double wheelBase = 0.595; // meters
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.45; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.1724 / 12); // TUNED
    public static final double driveKV = (2.0434 / 12);
    public static final double driveKA = (0.72651 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 5.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot

        public static final int driveMotorID = 8;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 4;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(118.65);
        
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot

        public static final int driveMotorID = 6;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55.27+180);

        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot

        public static final int driveMotorID = 2;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(113.104);

        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot

        public static final int driveMotorID = 4;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 2;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(5.09+180);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
}






public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2.0;
    public static final double kPYController = 2.0;
    public static final double kPThetaController = 3.3;

    public static HashMap<String, Command> eventMap = new HashMap<>();

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}





}
