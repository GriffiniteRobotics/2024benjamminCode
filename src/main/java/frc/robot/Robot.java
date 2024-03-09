// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.CANSWDLJNI;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
//import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(0,0);

  //PID for angular motion on limelight
  ProfiledPIDController driveController = new ProfiledPIDController(0.035,0, 0, profile);

  ProfiledPIDController yController = new ProfiledPIDController(.095, 0, .003, profile);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  PIDController pid = new PIDController(1, 0, 0);
  TalonFX intake = new TalonFX(13);
  VictorSPX feeder = new VictorSPX(14);
  TalonFX shooter1 = new TalonFX(15);
  TalonFX shooter2 = new TalonFX(16); //should always run SAME DIRECTION as shooter1
 
  AHRS navx = new AHRS(SPI.Port.kMXP);

  TalonFX climb1 = new TalonFX(17);
  TalonFX climb2 = new TalonFX(18);

  int dangerCount1;
  int dangerCount2;
  // declearing CANencoders
  /*
   * public CANcoder Back_left = new CANcoder(9);
   * public CANcoder Back_right = new CANcoder(10);
   * public CANcoder Front_left = new CANcoder(12);
   * public CANcoder Front_right = new CANcoder(11);
   */


  public static CANSparkBase.IdleMode brake = IdleMode.kBrake;

  Timer timer = new Timer();
  Timer Shooting_Timer = new Timer();
  Timer climb_timer = new Timer();
  Timer timer2 = new Timer();
  Timer timer3 = new Timer();
  Timer setpointTimer = new Timer();

  // Motors that control angle, ONE full rotation is about 12.8
  CANSparkMax FRONT_LEFT_ANGLE = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax FRONT_RIGHT_ANGLE = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax BACK_LEFT_ANGLE = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax BACK_RIGHT_ANGLE = new CANSparkMax(4, MotorType.kBrushless);

  // Motors that control speed
  CANSparkMax FRONT_LEFT_SPEED = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax FRONT_RIGHT_SPEED = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax BACK_LEFT_SPEED = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax BACK_RIGHT_SPEED = new CANSparkMax(3, MotorType.kBrushless);

  // Used only for absolute position
  // NEED TO RECHECK POSITIONS
  CANcoder FRONT_LEFT_CODER = new CANcoder(12);
  CANcoder FRONT_RIGHT_CODER = new CANcoder(11);
  CANcoder BACK_LEFT_CODER = new CANcoder(10);
  CANcoder BACK_RIGHT_CODER = new CANcoder(9);

  XboxController XBOX_ONE = new XboxController(0);
  //XboxController XBOX_TWO = new XboxController(1);

  // Locations for the swerve drive modules relative to the robot center.
  // Numbers don't really matter, just need to vaguely resemble the shape of the
  // robot
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  ChassisSpeeds speeds;

  double INIT_CLAMPED_YAW;
  double INIT_CLIMB1;
  double INIT_CLIMB2;
  XboxController gamepad = new XboxController(1);
  boolean Shooting = false;
  boolean note_in_low = false;
  boolean note_in_high = false;
  boolean atTarget = false;
  int atTargetcount = 0;
  DigitalInput IRLow = new DigitalInput(5);//False when note is present near intake
  DigitalInput IRHigh = new DigitalInput(4);//False when note is present near the shooter

  boolean RunIntake = true;



  // steps counter
  // int next_drive_step = 1;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    navx.reset();
    timer.reset();//what are these names?
    timer2.reset();
    timer3.reset();
    climb_timer.reset();
    Shooting = false;
    FRONT_LEFT_ANGLE.setIdleMode(IdleMode.kCoast);
    FRONT_RIGHT_ANGLE.setIdleMode(IdleMode.kCoast);
    BACK_LEFT_ANGLE.setIdleMode(IdleMode.kCoast);
    BACK_RIGHT_ANGLE.setIdleMode(IdleMode.kCoast);

    FRONT_LEFT_SPEED.setIdleMode(brake);
    FRONT_RIGHT_SPEED.setIdleMode(brake);
    BACK_LEFT_SPEED.setIdleMode(brake);
    BACK_RIGHT_SPEED.setIdleMode(brake);

    FRONT_LEFT_ANGLE.setSmartCurrentLimit(25);
    FRONT_RIGHT_ANGLE.setSmartCurrentLimit(25);
    BACK_LEFT_ANGLE.setSmartCurrentLimit(25);
    BACK_RIGHT_ANGLE.setSmartCurrentLimit(25);

    FRONT_LEFT_SPEED.setSmartCurrentLimit(35);
    FRONT_RIGHT_SPEED.setSmartCurrentLimit(35);
    BACK_LEFT_SPEED.setSmartCurrentLimit(35);
    BACK_RIGHT_SPEED.setSmartCurrentLimit(35);

    FRONT_LEFT_ANGLE.getEncoder().setPosition(0);
    FRONT_RIGHT_ANGLE.getEncoder().setPosition(0);
    BACK_LEFT_ANGLE.getEncoder().setPosition(0);
    BACK_RIGHT_ANGLE.getEncoder().setPosition(0);

    // PID THINGS
    FRONT_LEFT_ANGLE.getPIDController().setP(.1);
    FRONT_RIGHT_ANGLE.getPIDController().setP(.1);
    BACK_LEFT_ANGLE.getPIDController().setP(.1);
    BACK_RIGHT_ANGLE.getPIDController().setP(.1);


    INIT_CLIMB1 = climb1.getSelectedSensorPosition();
    INIT_CLIMB2 = climb2.getSelectedSensorPosition();
    

    System.out.println("Robot initialized");
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("timer", timer.get());
    SmartDashboard.putNumber("FRONT_LEFT_ANGLE", FRONT_LEFT_ANGLE.getEncoder().getPosition());
    SmartDashboard.putNumber("FRONT_LEFT_CODER", FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber("FRONT_RIGHT_ANGLE", FRONT_RIGHT_ANGLE.getEncoder().getPosition());
    SmartDashboard.putNumber("FRONT_RIGHT_CODER", FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber("BACK_LEFT_ANGLE", BACK_LEFT_ANGLE.getEncoder().getPosition());
    SmartDashboard.putNumber("BACK_LEFT_CODER", BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber("BACK_RIGHT_ANGLE", BACK_RIGHT_ANGLE.getEncoder().getPosition());
    SmartDashboard.putNumber("BACK_RIGHT_CODER", BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber("FRONT_LEFT_SPEED", FRONT_LEFT_SPEED.get());
    SmartDashboard.putNumber("FRONT_RIGHT_SPEED", FRONT_RIGHT_SPEED.get());

    SmartDashboard.putNumber("BACK_LEFT_SPEED", BACK_LEFT_SPEED.get());
    SmartDashboard.putNumber("BACK_RIGHT_SPEED", BACK_RIGHT_SPEED.get());
    SmartDashboard.putNumber("get.Yaw", navx.getYaw());
    SmartDashboard.putNumber("climb1.getSelectedSensorPosition()", climb1.getSelectedSensorPosition());
    SmartDashboard.putNumber("climb2.getSelectedSensorPosition()", climb2.getSelectedSensorPosition());
    SmartDashboard.putNumber("INIT_CLAMPED_YAW", INIT_CLAMPED_YAW);

    if(XBOX_ONE.getBackButton()){
    if(FRONT_LEFT_SPEED.get() == 0) {
      if (FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        FRONT_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        FRONT_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (FRONT_RIGHT_SPEED.get() == 0) {
      if (FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        FRONT_RIGHT_ANGLE.getEncoder()
            .setPosition(12.8 / 1 * FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        FRONT_RIGHT_ANGLE.getEncoder()
            .setPosition(12.8 / 1 * FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (BACK_LEFT_SPEED.get() == 0) {
      if (BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        BACK_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        BACK_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (BACK_RIGHT_SPEED.get() == 0) {
      if (BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        BACK_RIGHT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        BACK_RIGHT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }}
  }

  @Override
  public void autonomousInit() {
    Shooting = false;

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
    timer.reset();
    timer2.reset();
    timer3.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    if(FRONT_LEFT_SPEED.get() == 0) {
      if (FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        FRONT_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        FRONT_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (FRONT_RIGHT_SPEED.get() == 0) {
      if (FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        FRONT_RIGHT_ANGLE.getEncoder()
            .setPosition(12.8 / 1 * FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        FRONT_RIGHT_ANGLE.getEncoder()
            .setPosition(12.8 / 1 * FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (BACK_LEFT_SPEED.get() == 0) {
      if (BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        BACK_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        BACK_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (BACK_RIGHT_SPEED.get() == 0) {
      if (BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        BACK_RIGHT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        BACK_RIGHT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }

  

    if(IRLow.get() == false){
      note_in_low = true;
    }
    if(IRHigh.get() == false){
      note_in_high = true;
    }
    else{
      note_in_high = false;
    }

    if(note_in_high == true && Shooting==false){ //
     feeder.set(ControlMode.PercentOutput, 0);
    }

    if(note_in_low == false && note_in_high == false){
      intake.set(ControlMode.PercentOutput, .1);
    }

    if(note_in_low == true && IRLow.get() == true){//means the beam for the low IR sensor is not broken and there is a note within the bot
      intake.set(ControlMode.PercentOutput, 0); 
    }

    if(note_in_high == false && note_in_low == true){
      feeder.set(ControlMode.PercentOutput, .3);
    }

    if(note_in_high == true && Shooting == false){
      feeder.set(ControlMode.PercentOutput, 0);
    }

    if(note_in_low == true && note_in_high == true){
      note_in_low = false;
    }
    if(note_in_high && Shooting == true){
      shooter1.set(ControlMode.PercentOutput, 0.3);
      shooter2.set(ControlMode.PercentOutput, 0.3);
    }
    if(Shooting == false){ 
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
    }


    
    //ChassisSpeeds speeds;
    //lime rotational Value 
    double x = tx.getDouble(0);
    double y = ty.getDouble(0);

    double limeHasTarget = tv.getDouble(0);
    //at target value

    INIT_CLAMPED_YAW = Math.max(-0.1, Math.min(0.1, -navx.getYaw())); // needs work 2/6 changed to 0.1, -navx,
    // timers
    double setpoint = 0;
    double driveCommand = driveController.calculate(x, setpoint);
    SmartDashboard.putNumber("driveCommand", driveCommand);


    /*
     
     L = leave
     SL = shoot leave
     SLPRS = Shoot leave pickup note, return to speaker, shoot leave
     LTFS = Leave turn forward shoot (open feild)
     LTFSA Leave turn forward shoot (amp)

     */

   /*
     
     L = leave
     SL = shoot leave
     SLPRS = Shoot leave pickup note, return to speaker, shoot leave
     LTFS = Leave turn forward shoot (open feild)
     LTFSA Leave turn forward shoot (amp)

     */

     switch ("LTFS_R") {
      case "L":

        System.out.println("leave case is running");
        if(timer.get()< 0.5){
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(-navx.getYaw()));
        } else if (timer.get() < 4.5) {// 2.5 secs = 4 ft
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.6, 0, INIT_CLAMPED_YAW*2, Rotation2d.fromDegrees(-navx.getYaw()));
        } 

        else {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(-navx.getYaw()));
        }
        break;

      case "SL":
        System.out.println("shoot and leave");
        if (timer.get() < .5) {   
      System.out.println("step1 reving up to shoot");
      shooter1.set(ControlMode.PercentOutput, 0.5);
      shooter2.set(ControlMode.PercentOutput, 0.5);
      //Shooting_Timer.reset();
      Shooting = true;}
      else if (timer.get() >=.5 && timer.get() < 1.5 && Shooting == true){
      System.out.println("step2 Feeder and Shooting");
      feeder.set(ControlMode.PercentOutput, 0.3);
      shooter1.set(ControlMode.PercentOutput, 0.5);
      shooter2.set(ControlMode.PercentOutput, 0.5);
     }else if (timer.get() >= 1.5 && timer.get() <= 3.5 && Shooting == true) {
      System.out.println("Stopping");
      feeder.set(ControlMode.PercentOutput, 0);
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
        Shooting = false;
         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(1.2, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));//1.2
        }

        else if(timer.get() >= 3.5){
         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));

        }
        break;

      /*case "S":
        System.out.println("shoot and leave");
        if (timer.get() < .5) {   
      System.out.println("step1 reving up to shoot");
      shooter1.set(ControlMode.PercentOutput, 0.5);
      shooter2.set(ControlMode.PercentOutput, 0.5);
      //Shooting_Timer.reset();
      Shooting = true;}
    else if (timer.get() >=.5 && timer.get() < 1.5 && Shooting == true){
      System.out.println("step2 Feeder and Shooting");
      feeder.set(ControlMode.PercentOutput, 0.3);
      shooter1.set(ControlMode.PercentOutput, 0.5);
      shooter2.set(ControlMode.PercentOutput, 0.5);
    }else if (timer.get() >= 1.5 && timer.get() <= 3.5 && Shooting == true) {
      System.out.println("Stopping");
      feeder.set(ControlMode.PercentOutput, 0);
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
        Shooting = false;
         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(1.2, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));//1.2
        }

        else if(timer.get() >= 3.5){
         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));

        }
        break;*/

        case "SLPRS": //MOSTLIKY NOT GOING TO USE
        System.out.println("shoot leave, pickup note, return shoot, leave");
        if (timer.get() < 0.5 && note_in_high) {
          Shooting = true;
        } 

        else if(timer.get() >= 5){
          Shooting = false;
          timer2.reset();
          timer2.start();
        }

        else if (timer2.get() < 2.5) {
          if (navx.getYaw() > -175.5) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0,  Math.PI / 3, Rotation2d.fromDegrees(navx.getYaw()));
          } else speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
        }

        else if (navx.getYaw() > 4.5) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0,  -Math.PI / 3, Rotation2d.fromDegrees(navx.getYaw()));
          timer3.reset();
          timer3.start();
        }

       else if (timer3.get() < 2.5) {
         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
        }

        else if (timer3.get() > 2.5 && note_in_high) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
          Shooting = true;
        }

        else if (timer2.get() >= 3.5 && timer.get() <= 6) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
          Shooting = false;
        } 

        else if (timer.get() > 6) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw())); 
        }

        break;

      case "LTFS_R":
        if(timer.get() < 0.5){
           speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
           System.out.println("1stopped");
        }
        else if (timer.get() < 2.5) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(1.2, 0, 2*INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
          System.out.println("2leaving");
        } 
        
        else if (timer.get() < 2.6) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 2*INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
          System.out.println("3StoppingAgain");
        }

        else if (limeHasTarget == 0) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Math.PI / 5, Rotation2d.fromDegrees(navx.getYaw()));
          System.out.println("4Seeking");
        }

        else if (atTargetcount <= 5) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,-driveCommand, Rotation2d.fromDegrees(navx.getYaw()));
          System.out.println("5autoAim");
           if(x >= -4 && x <= 4){
            atTarget  = true;
            atTargetcount ++;
           }
              else{atTarget = false;
               }
            }

        else if(y < -1.5){ //need to change 5 with correct number
          System.out.println("6approach");
          speeds = new ChassisSpeeds(-0.4, 0.2, 0);
          timer2.reset();
          timer2.start();
        }

        else if(timer2.get() < 0.5 && note_in_high){
          speeds = new ChassisSpeeds(0, 0, 0);
          System.out.println("7shooting");
       System.out.println("step1 reving up to shoot");
      shooter1.set(ControlMode.PercentOutput, 0.5);
      shooter2.set(ControlMode.PercentOutput, 0.5);
      //Shooting_Timer.reset();
      Shooting = true;}
      else if (timer2.get() >=.5 && timer2.get() < 1.5 && Shooting == true){
      System.out.println("step2 Feeder and Shooting");
      feeder.set(ControlMode.PercentOutput, 0.3);
      shooter1.set(ControlMode.PercentOutput, 0.5);
      shooter2.set(ControlMode.PercentOutput, 0.5);
     }else if (timer2.get() >= 1.5 && timer2.get() <= 3.5 && Shooting == true) {
      System.out.println("Stopping");
      feeder.set(ControlMode.PercentOutput, 0);
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
        Shooting = false;
        }
        
        /*
         * else if(timer.get() >=7 & timer.get() < 9.5 ){
         * speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.2, -0.2, INIT_CLAMPED_YAW -
         * Math.PI/2, Rotation2d.fromDegrees(navx.getYaw()));
         * }
         */

      break;

        case "LTFS_L":
        
        if (timer.get() < 5) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.4, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
        } 
        
        else if (timer.get() >= 5 & timer.get() < 5.2) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
        }

        else if (timer.get() >= 5.2 & timer.get() < 5.7 & navx.getYaw() > 49) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Math.PI / 3, Rotation2d.fromDegrees(navx.getYaw()));
        }

        else if (atTargetcount <= 5) { 
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,driveCommand,Rotation2d.fromDegrees(-navx.getYaw()));
           if(x >= -.4 && x <= .4){
            atTarget  = true;
            atTargetcount ++;
           }
              else{atTarget = false;
               }
            }
        else if(y < 5){ //need to change 5 with correct number
          speeds = new ChassisSpeeds(0.3, 0, 0); 
          timer2.reset();
          timer2.start();
        }
        else if(timer2.get() < 0.5 && note_in_high){
         speeds = new ChassisSpeeds(0, 0, 0);
         Shooting = true;
        }

        else if(timer2.get() > 5){
          Shooting = false;
        }
        
      break;
         }
    

    

    // creating vec

    // copy here

    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    // Example: ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5); No navX
    
  
    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // module states, can pull angles and motor speed
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        Rotation2d.fromDegrees(360 / 12.8 * FRONT_LEFT_ANGLE.getEncoder().getPosition()));
    var frontRightOptimized = SwerveModuleState.optimize(frontRight,
        Rotation2d.fromDegrees(360 / 12.8 * FRONT_RIGHT_ANGLE.getEncoder().getPosition()));
    var backLeftOptimized = SwerveModuleState.optimize(backLeft,
        Rotation2d.fromDegrees(360 / 12.8 * BACK_LEFT_ANGLE.getEncoder().getPosition()));
    var backRightOptimized = SwerveModuleState.optimize(backRight,
        Rotation2d.fromDegrees(360 / 12.8 * BACK_RIGHT_ANGLE.getEncoder().getPosition()));

    // Should only set the angle position of each motor
    // Example: MOTOR_NAME.getPIDController().setReference(angle to encoder,
    // CANSparkMax.ControlType.kPosition);

    FRONT_LEFT_SPEED.set(frontLeftOptimized.speedMetersPerSecond / 4);
    FRONT_RIGHT_SPEED.set(frontRightOptimized.speedMetersPerSecond / 4);
    BACK_LEFT_SPEED.set(backLeftOptimized.speedMetersPerSecond / 4);
    BACK_RIGHT_SPEED.set(backRightOptimized.speedMetersPerSecond / 4);
    FRONT_LEFT_ANGLE.getPIDController().setReference(12.8 / 360 * frontLeftOptimized.angle.getDegrees(),
        CANSparkMax.ControlType.kPosition);
    FRONT_RIGHT_ANGLE.getPIDController().setReference(12.8 / 360 * frontRightOptimized.angle.getDegrees(),
        CANSparkMax.ControlType.kPosition);
    BACK_LEFT_ANGLE.getPIDController().setReference(12.8 / 360 * backLeftOptimized.angle.getDegrees(),
        CANSparkMax.ControlType.kPosition);
    BACK_RIGHT_ANGLE.getPIDController().setReference(12.8 / 360 * backRightOptimized.angle.getDegrees(),
        CANSparkMax.ControlType.kPosition);
    // end copy

    // Fun Dashboard Things
    SmartDashboard.putString("frontLeft: ", frontLeft.toString());
    SmartDashboard.putString("frontRight: ", frontRight.toString());
    SmartDashboard.putString("backLeft: ", backLeft.toString());
    SmartDashboard.putString("backRight: ", backRight.toString());
    SmartDashboard.putNumber("frontLeftVolt:", FRONT_LEFT_ANGLE.getOutputCurrent());
    SmartDashboard.putNumber("frontRightVolt:", FRONT_LEFT_ANGLE.getOutputCurrent());
    SmartDashboard.putNumber("backLeftVolt:", FRONT_LEFT_ANGLE.getOutputCurrent());
    SmartDashboard.putNumber("backRightVolt:", FRONT_LEFT_ANGLE.getOutputCurrent());
     
  }

  @Override
  public void teleopInit() {

  

    Shooting = false;

    /*if(FRONT_LEFT_SPEED.get() == 0) {
      if (FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        FRONT_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        FRONT_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * FRONT_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (FRONT_RIGHT_SPEED.get() == 0) {
      if (FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        FRONT_RIGHT_ANGLE.getEncoder()
            .setPosition(6.4 / .5 * FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        FRONT_RIGHT_ANGLE.getEncoder()
            .setPosition(6.4 / .5 * FRONT_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (BACK_LEFT_SPEED.get() == 0) {
      if (BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        BACK_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        BACK_LEFT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_LEFT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }
    if (BACK_RIGHT_SPEED.get() == 0) {
      if (BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble() > 0) {
        BACK_RIGHT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      } else {
        BACK_RIGHT_ANGLE.getEncoder().setPosition(6.4 / .5 * BACK_RIGHT_CODER.getAbsolutePosition().getValueAsDouble());
      }
    }*/

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
    timer.reset();
    timer.start();
    climb_timer.reset();

  }

  @Override
  public void teleopPeriodic() {

    //lime rotational value
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    //at target value
    boolean atTarget = false;

    double RIGHTX;
    double LEFTX;
    double LEFTY;

    if(Math.abs(XBOX_ONE.getRightX())  > 0.2){RIGHTX = XBOX_ONE.getRightX()*.8;
    }else{RIGHTX = 0;}

    if(Math.abs(XBOX_ONE.getLeftX())  > 0.2){LEFTX = XBOX_ONE.getLeftX();
    }else{LEFTX = 0;}

    if(Math.abs(XBOX_ONE.getLeftY())  > 0.2){LEFTY = XBOX_ONE.getLeftY();
    }else{LEFTY = 0;}

    if(XBOX_ONE.getAButton()){
      double setpoint = 0;
      //driveController.setTolerance(4,0.2);
 
      double driveCommand = driveController.calculate(x, setpoint);
      SmartDashboard.putNumber("driveCommand", driveCommand);
 
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-LEFTY, -LEFTX, -driveCommand, Rotation2d.fromDegrees(-navx.getYaw()));
      /*if(x >= -.4 && x <= .4){
        atTarget = true;
         
        setpointTimer.start();
        SmartDashboard.putNumber("Timer", setpointTimer.get());
        //driveController.setTolerance(0);
      }else{atTarget = false;}*/
    }else{
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-LEFTY, -LEFTX, RIGHTX, Rotation2d.fromDegrees(-navx.getYaw()));
      setpointTimer.stop();
      setpointTimer.reset();
    }
 
    /*SmartDashboard.putBoolean("At Target?", atTarget);
 
 
    if ( setpointTimer.get() <= 3 && setpointTimer.get() >= 2){
      setpointTimer.stop();
   
      double setpointY = -1.5;
      double yDriveCommand = yController.calculate(y, setpointY);
      SmartDashboard.putNumber("YdriveCommand", yDriveCommand);
 
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(yDriveCommand, -LEFTX, RIGHTX, Rotation2d.fromDegrees(-navx.getYaw()));
 
    }*/

    //Shooting_Timer.reset();
    //intake smartdashboard
    
    if(IRLow.get() == false){
      note_in_low = true;
    }
    if(IRHigh.get() == false){
      note_in_high = true;
    }
    else{
      note_in_high = false;
    }

    if(XBOX_ONE.getLeftBumperPressed()){
      RunIntake = !RunIntake;
    }

    if(XBOX_ONE.getYButton()){
      if(IRLow.get() == true){
        feeder.set(ControlMode.PercentOutput, -.3);
        shooter1.set(ControlMode.PercentOutput, -0.4);
        shooter2.set(ControlMode.PercentOutput, -0.4);
      }else{
        feeder.set(ControlMode.PercentOutput, 0);
        shooter1.set(ControlMode.PercentOutput, 0);
        shooter2.set(ControlMode.PercentOutput, 0);
      }
    }else{
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      if((note_in_high == true && Shooting==false) || note_in_high == false){
        feeder.set(ControlMode.PercentOutput, 0);
      }

      if(note_in_low == false && note_in_high == false){
        if(RunIntake){
          intake.set(ControlMode.PercentOutput, -.36);//-.26
          XBOX_ONE.setRumble(RumbleType.kBothRumble,.5);
        }else{
          intake.set(ControlMode.PercentOutput, 0);
          XBOX_ONE.setRumble(RumbleType.kBothRumble,0);
        }
      }

      /*if(note_in_low == true && IRLow.get() == true){//means the beam for the low IR sensor is not broken and there is a note within the bot
        intake.set(ControlMode.PercentOutput, 0);
        feeder.set(ControlMode.PercentOutput, .4);
      }*/

      if(note_in_high == false && note_in_low == true){
        feeder.set(ControlMode.PercentOutput, .6);
      }

      if(note_in_low == true && IRLow.get() == true){//means the beam for the low IR sensor is not broken and there is a note within the bot
        intake.set(ControlMode.PercentOutput, 0);
        feeder.set(ControlMode.PercentOutput, .4);
      }

      if(note_in_high == true && Shooting == false){
        feeder.set(ControlMode.PercentOutput, 0);
      }

      if(note_in_low == true && note_in_high == true){
        note_in_low = false;
      }

      if(note_in_high && Shooting == true){
        shooter1.set(ControlMode.PercentOutput, 0.3);
        shooter2.set(ControlMode.PercentOutput, 0.3);
      }
      if(Shooting == false){ 
        shooter1.set(ControlMode.PercentOutput, 0);
        shooter2.set(ControlMode.PercentOutput, 0);
      }
    }

    SmartDashboard.putBoolean("IRLow.get()", IRLow.get());
    SmartDashboard.putBoolean("IRHigh.get()", IRHigh.get());
    SmartDashboard.putNumber("Dpad", gamepad.getPOV());
    //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(LEFTY, LEFTX, RIGHTX, Rotation2d.fromDegrees(-navx.getYaw()));

    //Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    //module states, can pull angles and motor speed
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];
    
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, Rotation2d.fromDegrees(360/12.8*FRONT_LEFT_ANGLE.getEncoder().getPosition()));
    var frontRightOptimized = SwerveModuleState.optimize(frontRight, Rotation2d.fromDegrees(360/12.8*FRONT_RIGHT_ANGLE.getEncoder().getPosition()));
    var backLeftOptimized = SwerveModuleState.optimize(backLeft, Rotation2d.fromDegrees(360/12.8*BACK_LEFT_ANGLE.getEncoder().getPosition()));
    var backRightOptimized = SwerveModuleState.optimize(backRight, Rotation2d.fromDegrees(360/12.8*BACK_RIGHT_ANGLE.getEncoder().getPosition()));

    //Should only set the angle position of each motor
    //Example: MOTOR_NAME.getPIDController().setReference(angle to encoder, CANSparkMax.ControlType.kPosition);
    if(XBOX_ONE.getLeftTriggerAxis() > 0.2){
      shooter1.set(ControlMode.PercentOutput, XBOX_ONE.getLeftTriggerAxis());
      shooter2.set(ControlMode.PercentOutput, XBOX_ONE.getLeftTriggerAxis());
      feeder.set(ControlMode.PercentOutput, XBOX_ONE.getLeftTriggerAxis());
    }

    if(XBOX_ONE.getRightTriggerAxis() > .2){
      FRONT_LEFT_SPEED.set(frontLeft.speedMetersPerSecond*.2);
      FRONT_RIGHT_SPEED.set(frontRight.speedMetersPerSecond*.2);
      BACK_LEFT_SPEED.set(backLeft.speedMetersPerSecond*.2);
      BACK_RIGHT_SPEED.set(backRight.speedMetersPerSecond*.2);
      FRONT_LEFT_ANGLE.getPIDController().setReference(12.8/360*frontLeft.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
      FRONT_RIGHT_ANGLE.getPIDController().setReference(12.8/360*frontRight.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
      BACK_LEFT_ANGLE.getPIDController().setReference(12.8/360*backLeft.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
      BACK_RIGHT_ANGLE.getPIDController().setReference(12.8/360*backRight.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }else{
      FRONT_LEFT_SPEED.set(frontLeftOptimized.speedMetersPerSecond*.9);
      FRONT_RIGHT_SPEED.set(frontRightOptimized.speedMetersPerSecond*.9);
      BACK_LEFT_SPEED.set(backLeftOptimized.speedMetersPerSecond*.9);
      BACK_RIGHT_SPEED.set(backRightOptimized.speedMetersPerSecond*.9);
      FRONT_LEFT_ANGLE.getPIDController().setReference(12.8/360*frontLeftOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
      FRONT_RIGHT_ANGLE.getPIDController().setReference(12.8/360*frontRightOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
      BACK_LEFT_ANGLE.getPIDController().setReference(12.8/360*backLeftOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
      BACK_RIGHT_ANGLE.getPIDController().setReference(12.8/360*backRightOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }
    
    //end copy

    // intake steps

    /*if(XBOX_ONE.getAButton()){
      double setpoint = 0;
      driveController.setTolerance(4);

      FRONT_LEFT_ANGLE.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
      FRONT_RIGHT_ANGLE.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
      BACK_LEFT_ANGLE.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
      BACK_RIGHT_ANGLE.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
      //double driveCommand = MathUtil.clamp(driveController.calculate(x, setpoint), -1, 1);


      double driveCommand = driveController.calculate(x, setpoint);
      SmartDashboard.putNumber("driveCommand", driveCommand);
      //left1.set(driveCommand);
      //right1.set(driveCommand);
      //left1.set(-driveCommand);
      //right1.set(-driveCommand);

      FRONT_LEFT_SPEED.setVoltage(-driveCommand);
      FRONT_RIGHT_SPEED.setVoltage(driveCommand);
      //TOP_RIGHT_SPEED.set(driveCommand);
      //TOP_LEFT_SPEED.set(driveCommand);
    }else{// is this needed?
      /*FRONT_LEFT_SPEED.set(-XBOX_ONE.getLeftY());
      FRONT_RIGHT_SPEED.set(XBOX_ONE.getRightY());
    }*/

    if(XBOX_ONE.getXButtonPressed() == true && Shooting == false && note_in_high){
      System.out.println("step1 reving up to shoot");
      shooter1.set(ControlMode.PercentOutput, 0.7);
      shooter2.set(ControlMode.PercentOutput, 0.7);
      Shooting_Timer.reset();
      Shooting_Timer.start();
      //Shooting_Timer.reset();
      Shooting = true;
    }else if (Shooting_Timer.get() >=.5 && Shooting_Timer.get() < 2.25 && Shooting == true){
      System.out.println("step2 Feeder and Shooting");
      feeder.set(ControlMode.PercentOutput, 0.4);
      shooter1.set(ControlMode.PercentOutput, 0.7);
      shooter2.set(ControlMode.PercentOutput, 0.7);
    }else if (Shooting_Timer.get() >= 2.25 && Shooting == true) {
      System.out.println("Stopping");
      feeder.set(ControlMode.PercentOutput, 0);
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      Shooting = false;
    }


    //arm code/climber code
    /*if(Math.abs(ClimbRight.getMotorOutputPercent()) > .25 && Math.abs(ClimbRight.getActiveTrajectoryVelocity()) < 10)
  ++ dangerCount1; 
  if(dangerCount1 > 15)
  ClimbRight.set(ControlMode.PercentOutput, 0);
  
  else if(dangerCount1 == 13)
  System.out.println("Danger danger !!!!!!!!!!!!!!!!count 1");
  
  if(Math.abs(ClimbLeft.getMotorOutputPercent()) > .25 && Math.abs(ClimbLeft.getActiveTrajectoryVelocity()) < 10)
  ++ dangerCount2; 
  if(dangerCount2 > 15)
  ClimbLeft.set(ControlMode.PercentOutput, 0);
  
  else if(dangerCount2 == 13)
  System.out.println("Danger danger !!!!!!!!!!!!!!!!count 2");*/





    if(gamepad.getLeftBumper() && climb1.getSelectedSensorPosition()<=165000){ //103248.000000
      climb1.set(ControlMode.PercentOutput,0.8);
    }else{
      climb1.set(ControlMode.PercentOutput,0);
    }
    if( gamepad.getLeftTriggerAxis() > 0.2 && climb1.getSelectedSensorPosition()>=0){//INIT_CLIMB1
      System.out.println("Climbing 1...");
      climb1.set(ControlMode.PercentOutput,-0.7);
    }

    if(gamepad.getRightBumper() && climb2.getSelectedSensorPosition()<=180000){//95692.000000//183560
      climb2.set(ControlMode.PercentOutput,0.8);
    }else{
      climb2.set(ControlMode.PercentOutput,0);
    }
    if(gamepad.getRightTriggerAxis() > 0.2 && climb2.getSelectedSensorPosition()>=0){//INIT_CLIMB2
      climb2.set(ControlMode.PercentOutput,-0.7);
      System.out.println("Climbing 2...");
    }

    if(gamepad.getPOV() == 0){
      climb1.set(ControlMode.PercentOutput,0.3);
      climb2.set(ControlMode.PercentOutput,-0.3);
    }else if (gamepad.getPOV() == 180) {
      climb1.set(ControlMode.PercentOutput,-0.3);
      climb2.set(ControlMode.PercentOutput,0.3);
    }/*else{
      climb1.set(ControlMode.PercentOutput,0);
      climb2.set(ControlMode.PercentOutput,0);
    }*/

    if(gamepad.getYButtonPressed() && gamepad.getBButtonPressed() && gamepad.getPOV() == -1 && climb2.getSelectedSensorPosition() == 90/Math.PI){
      if (navx.getRoll() <= -10) {
        climb1.set(ControlMode.PercentOutput,-0.05);
        climb2.set(ControlMode.PercentOutput,0.05);
      }else if (navx.getRoll() >= 10) {
        climb1.set(ControlMode.PercentOutput,0.05);
        climb2.set(ControlMode.PercentOutput,-0.05);
      }else if(navx.getRoll() >= -10 && navx.getRoll() <= 10){
        climb1.set(ControlMode.PercentOutput,0);
        climb2.set(ControlMode.PercentOutput,0);
      }
    }

    if(XBOX_ONE.getBButton()) { // amp shooting
      shooter1.set(ControlMode.PercentOutput,0.19);
      shooter2.set(ControlMode.PercentOutput,0.19);
      feeder.set(ControlMode.PercentOutput,0.2);

      if(note_in_high = false) {
        shooter1.set(ControlMode.PercentOutput,0.0);
        shooter2.set(ControlMode.PercentOutput,0.0);
      }
    }

  }

  @Override
  public void disabledInit() {
    Shooting = false;
  }

  @Override
  public void disabledPeriodic() {
    // resetting var (testing purposes)
    Shooting = false;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
    feeder.set(ControlMode.PercentOutput, 0.0);
    intake.set(ControlMode.PercentOutput, 0.0);
    shooter1.set(ControlMode.PercentOutput, 0.0);
    shooter2.set(ControlMode.PercentOutput, 0.0);
    climb1.setSelectedSensorPosition(0);
    climb2.setSelectedSensorPosition(0);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    if(gamepad.getLeftBumper()){ //103248.000000
      climb1.set(ControlMode.PercentOutput,0.3);
    }else if( gamepad.getLeftTriggerAxis() > 0.2){//INIT_CLIMB1
      System.out.println("Climbing 1...");
      climb1.set(ControlMode.PercentOutput,-0.3);
    }else{
      climb1.set(ControlMode.PercentOutput,0);
    }

    if(gamepad.getRightTriggerAxis() > 0.2){//INIT_CLIMB2
      climb2.set(ControlMode.PercentOutput,-0.3);
      System.out.println("Climbing 2...");
    }else if(gamepad.getRightBumper()){//95692.000000//183560
      climb2.set(ControlMode.PercentOutput,0.3);
    }else{
      climb2.set(ControlMode.PercentOutput,0);
    }
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
