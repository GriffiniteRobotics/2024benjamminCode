// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
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
  ProfiledPIDController driveController = new ProfiledPIDController(.095,.075, 0, profile);

  ProfiledPIDController yController = new ProfiledPIDController(.095, 0, .003, profile);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  PIDController pid = new PIDController(1, 0, 0);
  VictorSPX intake = new VictorSPX(13);
  VictorSPX feeder = new VictorSPX(14);
  TalonFX shooter1 = new TalonFX(15);
  TalonFX shooter2 = new TalonFX(16); //should always run SAME DIRECTION as shooter1
 
  AHRS navx = new AHRS(SPI.Port.kMXP);

  TalonFX climb1 = new TalonFX(17);
  TalonFX climb2 = new TalonFX(18);
  // declearing CANencoders
  /*
   * public CANcoder Back_left = new CANcoder(9);
   * public CANcoder Back_right = new CANcoder(10);
   * public CANcoder Front_left = new CANcoder(12);
   * public CANcoder Front_right = new CANcoder(11);
   */

  // declearing encoders

  public RelativeEncoder frist_Spark_encoder;

  public RelativeEncoder second_Spark_encoder;

  public RelativeEncoder third_Spark_encoder;

  public RelativeEncoder fouth_Spark_encoder;

  // Angle Motor
  public RelativeEncoder fifth_Spark_encoder;

  public RelativeEncoder six_Spark_encoder;

  public RelativeEncoder seventh_Spark_encoder;

  public RelativeEncoder eighth_Spark_encoder;

  public static CANSparkBase.IdleMode brake = IdleMode.kBrake;

  Timer timer;
  Timer Shooting_Timer = new Timer();
  Timer climb_timer = new Timer();
  Timer timer2 = new Timer();
  Timer timer3 = new Timer();
  Timer setpointTimer = new Timer();

  // Motors that control angle, ONE full rotation is about 12.8
  CANSparkMax FRONT_LEFT_ANGLE = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax FRONT_RIGHT_ANGLE = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax BACK_LEFT_ANGLE = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax BACK_RIGHT_ANGLE = new CANSparkMax(3, MotorType.kBrushless);

  // Motors that control speed
  CANSparkMax FRONT_LEFT_SPEED = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax FRONT_RIGHT_SPEED = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax BACK_LEFT_SPEED = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax BACK_RIGHT_SPEED = new CANSparkMax(4, MotorType.kBrushless);

  // Used only for absolute position
  // NEED TO RECHECK POSITIONS
  CANcoder FRONT_LEFT_CODER = new CANcoder(12);
  CANcoder FRONT_RIGHT_CODER = new CANcoder(11);
  CANcoder BACK_LEFT_CODER = new CANcoder(9);
  CANcoder BACK_RIGHT_CODER = new CANcoder(10);

  XboxController XBOX_ONE = new XboxController(0);

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
  XboxController gamepad = new XboxController(0);
  boolean Shooting = false;
  boolean note_in_low = false;
  boolean note_in_high = false;
  boolean atTarget = false;
  int atTargetcount = 0;
  DigitalInput IRLow = new DigitalInput(5);//False when note is present near intake
  DigitalInput IRHigh = new DigitalInput(4);//False when note is present near the shooter



  // steps counter
  // int next_drive_step = 1;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
 timer.reset();
 timer2.reset();
 timer3.reset();
 climb_timer.reset();
 Shooting_Timer.reset();
 Shooting = false;
    FRONT_LEFT_ANGLE.setIdleMode(brake);
    FRONT_RIGHT_ANGLE.setIdleMode(brake);
    BACK_LEFT_ANGLE.setIdleMode(brake);
    BACK_RIGHT_ANGLE.setIdleMode(brake);

    FRONT_LEFT_SPEED.setIdleMode(brake);
    FRONT_RIGHT_SPEED.setIdleMode(brake);
    BACK_LEFT_SPEED.setIdleMode(brake);
    BACK_RIGHT_SPEED.setIdleMode(brake);

    FRONT_LEFT_ANGLE.setSmartCurrentLimit(25);
    FRONT_RIGHT_ANGLE.setSmartCurrentLimit(25);
    BACK_LEFT_ANGLE.setSmartCurrentLimit(25);
    BACK_RIGHT_ANGLE.setSmartCurrentLimit(25);

    FRONT_LEFT_ANGLE.getEncoder().setPosition(0);
    FRONT_RIGHT_ANGLE.getEncoder().setPosition(0);
    BACK_LEFT_ANGLE.getEncoder().setPosition(0);
    BACK_RIGHT_ANGLE.getEncoder().setPosition(0);

    // PID THINGS
    FRONT_LEFT_ANGLE.getPIDController().setP(.1);
    FRONT_RIGHT_ANGLE.getPIDController().setP(.1);
    BACK_LEFT_ANGLE.getPIDController().setP(.1);
    BACK_RIGHT_ANGLE.getPIDController().setP(.1);

    navx.reset();
    timer = new Timer();

    // FRONT_LEFT_SPEED.addFollower(FRONT_RIGHT_SPEED);
    // BACK_LEFT_SPEED.addFollower(BACK_RIGHT_SPEED);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("timer", timer.get());
    SmartDashboard.putNumber("Angle 1", FRONT_LEFT_ANGLE.getEncoder().getPosition());
    SmartDashboard.putNumber("Angle 2", FRONT_RIGHT_ANGLE.getEncoder().getPosition());

    SmartDashboard.putNumber("Angle 3", BACK_LEFT_ANGLE.getEncoder().getPosition());
    SmartDashboard.putNumber("Angle 4", BACK_RIGHT_ANGLE.getEncoder().getPosition());

    SmartDashboard.putNumber("drive 1", FRONT_LEFT_SPEED.getEncoder().getPosition());
    SmartDashboard.putNumber("drive 2", FRONT_RIGHT_SPEED.getEncoder().getPosition());

    SmartDashboard.putNumber("drive 3", BACK_LEFT_SPEED.getEncoder().getPosition());
    SmartDashboard.putNumber("drive 4", BACK_RIGHT_SPEED.getEncoder().getPosition());
    SmartDashboard.putNumber("get.Yaw", navx.getYaw());
  }

  @Override
  public void autonomousInit() {
    Shooting = false;

    if (FRONT_LEFT_SPEED.get() == 0) {
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
    }

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
    timer.reset();
    timer2.reset();
    timer3.reset();
    timer.start();
    navx.reset();
  }

  @Override
  public void autonomousPeriodic() {

  

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
      Shooting_Timer.start();
      shooter1.set(ControlMode.PercentOutput, 0.3);
      shooter2.set(ControlMode.PercentOutput, 0.3);
      if(Shooting_Timer.get() > 0.5){
      shooter1.set(ControlMode.PercentOutput, 0.3);
      shooter2.set(ControlMode.PercentOutput, 0.3);
      feeder.set(ControlMode.PercentOutput, 0.3);
      }
    }
    if(Shooting == false){ 
      shooter1.set(ControlMode.PercentOutput, 0);
      shooter2.set(ControlMode.PercentOutput, 0);
      Shooting_Timer.reset();

    }


    
    //ChassisSpeeds speeds;
    //lime rotational Value 
    double x = tx.getDouble(0);
    double y = ty.getDouble(0);
    //at target value

    INIT_CLAMPED_YAW = Math.max(-0.05, Math.min(0.05, navx.getYaw())); // needs work 2/6
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

    switch ("Back_Turn") {
      case "L":

        System.out.println("leave case is running");
        if (timer.get() < 5) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.4, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
        } 

        else {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
        }
        break;

      case "SL":
        System.out.println("shoot and leave");
        if (timer.get() < 2 && note_in_high) {
          Shooting = true;
        } 

        else if (timer.get() >= 5) {
          Shooting = false;
         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.3, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
        }
        break;

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

      case "BTFS (speaker side)":

        if (timer.get() < 5) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.4, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
        } 
        
        else if (timer.get() < 5.2) {
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, INIT_CLAMPED_YAW, Rotation2d.fromDegrees(navx.getYaw()));
        }

        else if (navx.getYaw() > -41) {
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

        else if(timer2.get() > 1){
        Shooting = false;
        }
        
        /*
         * else if(timer.get() >=7 & timer.get() < 9.5 ){
         * speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.2, -0.2, INIT_CLAMPED_YAW -
         * Math.PI/2, Rotation2d.fromDegrees(navx.getYaw()));
         * }
         */

      break;

        case "BTFSA":
        
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
    navx.reset();
    Shooting = false;

    if (FRONT_LEFT_SPEED.get() == 0) {
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
    }

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(navx.getYaw()));
    timer.reset();
    timer.start();
    navx.reset();
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

    if(Math.abs(XBOX_ONE.getRightX())  > 0.2){RIGHTX = XBOX_ONE.getRightX();}else{RIGHTX = 0;}
    if(Math.abs(XBOX_ONE.getLeftX())  > 0.2){LEFTX = XBOX_ONE.getLeftX();}else{LEFTX = 0;}
    if(Math.abs(XBOX_ONE.getLeftY())  > 0.2){LEFTY = XBOX_ONE.getLeftY();}else{LEFTY = 0;}

    if(XBOX_ONE.getAButton()){double setpoint = 0;
      //driveController.setTolerance(4,0.2);
 
       double driveCommand = driveController.calculate(x, setpoint);
       SmartDashboard.putNumber("driveCommand", driveCommand);
 
       speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-LEFTY, -LEFTX, driveCommand, Rotation2d.fromDegrees(-navx.getYaw()));
       if(x >= -.4 && x <= .4){
         atTarget = true;
         
         setpointTimer.start();
         SmartDashboard.putNumber("Timer", setpointTimer.get());
         //driveController.setTolerance(0);
       }
       
        else{atTarget = false;}
         }
       
     else{ speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-LEFTY, -LEFTX, RIGHTX, Rotation2d.fromDegrees(-navx.getYaw()));
           setpointTimer.stop();
           setpointTimer.reset();
       }
 
       SmartDashboard.putBoolean("At Target?", atTarget);
 
 
       if ( setpointTimer.get() <= 3 && setpointTimer.get() >= 2){
       setpointTimer.stop();
   
       double setpointY = -1.5;
       double yDriveCommand = yController.calculate(y, setpointY);
       SmartDashboard.putNumber("YdriveCommand", yDriveCommand);
 
       speeds = ChassisSpeeds.fromFieldRelativeSpeeds(yDriveCommand, -LEFTX, RIGHTX, Rotation2d.fromDegrees(-navx.getYaw()));
       
 
 
       }

  //Shooting_Timer.reset();
 // intake smartdashboard

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

   SmartDashboard.putNumber("Dpad", gamepad.getPOV());
  speeds = ChassisSpeeds.fromFieldRelativeSpeeds(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX(), Rotation2d.fromDegrees(navx.getYaw()));

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

    FRONT_LEFT_SPEED.set(frontLeftOptimized.speedMetersPerSecond/4);
    FRONT_RIGHT_SPEED.set(frontRightOptimized.speedMetersPerSecond/4);
    BACK_LEFT_SPEED.set(backLeftOptimized.speedMetersPerSecond/4);
    BACK_RIGHT_SPEED.set(backRightOptimized.speedMetersPerSecond/4);
    FRONT_LEFT_ANGLE.getPIDController().setReference(12.8/360*frontLeftOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    FRONT_RIGHT_ANGLE.getPIDController().setReference(12.8/360*frontRightOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    BACK_LEFT_ANGLE.getPIDController().setReference(12.8/360*backLeftOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    BACK_RIGHT_ANGLE.getPIDController().setReference(12.8/360*backRightOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    //end copy

   // intake steps

   if (XBOX_ONE.getAButton()){
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
} else{
      FRONT_LEFT_SPEED.set(-XBOX_ONE.getLeftY());
      FRONT_RIGHT_SPEED.set(XBOX_ONE.getRightY());}

  if(XBOX_ONE.getXButtonPressed() == true && Shooting == false && note_in_high){
         
    System.out.println("step1 reving up to shoot");
    shooter1.set(ControlMode.PercentOutput, 0.2);
    shooter2.set(ControlMode.PercentOutput, 0.2);
    Shooting_Timer.reset();
    Shooting_Timer.start();
    //Shooting_Timer.reset();
 Shooting = true;
}
else if (Shooting_Timer.get() >=2 && Shooting_Timer.get() < 5 && Shooting == true)  {
System.out.println("step2 Feeder and Shooting");
feeder.set(ControlMode.PercentOutput, 0.3);
shooter1.set(ControlMode.PercentOutput, 0.3);
shooter2.set(ControlMode.PercentOutput, 0.3);

}
else if (Shooting_Timer.get() >= 5 && Shooting == true) {
System.out.println("Stopping");
 feeder.set(ControlMode.PercentOutput, 0);
shooter1.set(ControlMode.PercentOutput, 0);
shooter2.set(ControlMode.PercentOutput, 0);
Shooting = false;

}
  //arm code/climber code
  if(gamepad.getLeftTriggerAxis() > 0.2){
    climb1.set(ControlMode.PercentOutput,0.3);
  }

  else if(gamepad.getLeftBumperPressed()){
  climb1.set(ControlMode.PercentOutput,-0.3);
  }

  if(gamepad.getRightTriggerAxis() > 0.2 ){
    climb2.set(ControlMode.PercentOutput,-0.3);
  }

  else if(gamepad.getRightBumperPressed()){
   climb2.set(ControlMode.PercentOutput,0.3);
  }

 if (gamepad.getPOV() == 0){
  climb1.set(ControlMode.PercentOutput,0.3);
  climb2.set(ControlMode.PercentOutput,-0.3);
 }

 else if (gamepad.getPOV() == 180) {
  climb1.set(ControlMode.PercentOutput,-0.3);
  climb2.set(ControlMode.PercentOutput,0.3);
 }

else if(gamepad.getYButtonPressed() && gamepad.getBButtonPressed() && gamepad.getPOV() == -1 && climb2.getSelectedSensorPosition() == 90/Math.PI){
  if (navx.getRoll() <= -10) {
    climb1.set(ControlMode.PercentOutput,-0.05);
    climb2.set(ControlMode.PercentOutput,0.05);
  }

  else if (navx.getRoll() >= 10) {
    climb1.set(ControlMode.PercentOutput,0.05);
    climb2.set(ControlMode.PercentOutput,-0.05);
  }

  else if(navx.getRoll() >= -10 && navx.getRoll() <= 10){
   climb1.set(ControlMode.PercentOutput,0);
   climb2.set(ControlMode.PercentOutput,0);
  }
}

if (gamepad.getBButtonPressed() && gamepad.getYButtonPressed() == false) { // amp shooting
  shooter1.set(ControlMode.PercentOutput,0.15);
  shooter2.set(ControlMode.PercentOutput,0.15);
  feeder.set(ControlMode.PercentOutput,0.15);

  if (note_in_high = false) {
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
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    // only run when motors are straight
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
