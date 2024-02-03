// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class BasePilotable_SS extends SubsystemBase {



private final CANSparkMax m_LeftMotor1 = new CANSparkMax(Constants.BPConstants.kLeftMotorPort1, MotorType.kBrushless);
private final CANSparkMax m_LeftMotor2 = new CANSparkMax(Constants.BPConstants.kLeftMotorPort2, MotorType.kBrushless);
private final CANSparkMax m_RightMotor3 = new CANSparkMax(Constants.BPConstants.kRightMotorPort3, MotorType.kBrushless);
private final CANSparkMax m_RightMotor4 = new CANSparkMax(Constants.BPConstants.kRightMotorPort4, MotorType.kBrushless);


  private final DifferentialDrive  m_DifferentialDrive = new DifferentialDrive(m_LeftMotor1, m_RightMotor3);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);
     

  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

// Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

   private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics( DriveConstants.kTrackwidthMeters);
 // Simulation classes help us simulate our robot
  
 public DifferentialDrivetrainSim m_drivetrainSimulator;
 private final EncoderSim m_leftEncoderSim;
 private final EncoderSim m_rightEncoderSim;
 // The Field2d class shows the field in the sim GUI
 private final Field2d m_fieldSim;
 private final ADXRS450_GyroSim m_gyroSim;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
//private Field2d m_field;


ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
DifferentialDriveWheelSpeeds wheelSpeeds =
m_kinematics.toWheelSpeeds(chassisSpeeds);

// wheel velocities
double leftVelocity = wheelSpeeds.leftMetersPerSecond;
double rightVelocity = wheelSpeeds.rightMetersPerSecond;
ChassisSpeeds speedsDIFF = new ChassisSpeeds(1.0, 0.0, 1.5);



  /** Creates a new BasePilotable_SS. */
  public BasePilotable_SS() {

 SendableRegistry.addChild(m_DifferentialDrive, m_LeftMotor1);
    SendableRegistry.addChild(m_DifferentialDrive, m_RightMotor3);

    m_LeftMotor1.restoreFactoryDefaults();
    m_LeftMotor2.restoreFactoryDefaults();
    m_RightMotor3.restoreFactoryDefaults();
    m_RightMotor4.restoreFactoryDefaults();

    m_RightMotor3.setInverted(true);
    m_RightMotor4.setInverted(true);

    m_LeftMotor2.follow(m_LeftMotor1);
    m_RightMotor4.follow(m_RightMotor3); 


 // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()),
            m_leftEncoder.getDistance(),
            m_rightEncoder.getDistance());


 if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_gyroSim = new ADXRS450_GyroSim(m_gyro);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    } else {
      m_leftEncoderSim = null;
      m_rightEncoderSim = null;
      m_gyroSim = null;

      m_fieldSim = null;
    }




      
  // Configure AutoBuilder last
        AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // Current ChassisSpeeds supplier
                this::toWheelSpeeds, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance());
  m_fieldSim.setRobotPose(getPose());
  }

/** Update our simulation. This should be run every robot loop in simulation. */
 @Override
 public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_LeftMotor1.get() * RobotController.getInputVoltage(),
        m_RightMotor3.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }




   /** Sets speeds to the drivetrain motors. */
   public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_LeftMotor1.setVoltage(leftOutput + leftFeedforward);
    m_RightMotor3.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

 public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }



  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        pose);
  }






public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

 public void arcadeDrive(double fwd, double rot){

    m_DifferentialDrive.arcadeDrive(fwd, rot);

  }


 public void basePilotableStop(){

    m_DifferentialDrive.stopMotor();

  }


  public void tankDrive(double leftSpeed, double rightSpeed){

    m_DifferentialDrive.tankDrive(leftSpeed, rightSpeed);

  }


  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_LeftMotor1.setVoltage(leftVolts);
    m_RightMotor3.setVoltage(rightVolts);
    m_DifferentialDrive.feed();
  }




    public DifferentialDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds)
  {
    return m_kinematics.toWheelSpeeds(chassisSpeeds);
  }

  public ChassisSpeeds toChassisSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
       return new ChassisSpeeds(
          (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2,0,
            (wheelSpeeds.rightMetersPerSecond - wheelSpeeds.leftMetersPerSecond) /  DriveConstants.kTrackwidthMeters);
     }
    
  public ChassisSpeeds toChassisSpeeds2(DifferentialDriveWheelSpeeds wheelSpeeds) {
       return new ChassisSpeeds(((wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond )/2) / DriveConstants.kTrackwidthMeters, 0, 0);
     }
    



public DifferentialDriveWheelSpeeds toWheelSpeeds2(ChassisSpeeds chassisSpeeds) {
   return new DifferentialDriveWheelSpeeds(
        chassisSpeeds.vxMetersPerSecond
            - DriveConstants.kTrackwidthMeters / 2 * chassisSpeeds.omegaRadiansPerSecond,
       chassisSpeeds.vxMetersPerSecond
           + DriveConstants.kTrackwidthMeters/ 2 * chassisSpeeds.omegaRadiansPerSecond);  }




           public double getLeftEncoderVelocity() {
            return m_LeftMotor1.getEncoder().getVelocity() ;
          }
          public double getRightEncoderVelocity() {
            return m_RightMotor3.getEncoder().getVelocity() ;
          }



   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate() , m_rightEncoder.getRate() );
  }


  
   //C'est la bonne méthode pour convertir la vitesse en metres secondes avec les encodeurs NEO d'intégrés
   public DifferentialDriveWheelSpeeds  getSpeeds2(){
    return new DifferentialDriveWheelSpeeds(
      m_LeftMotor1.getEncoder().getVelocity() /  (10.71) * Math.PI *  (0.1524) / 60,
    m_RightMotor3.getEncoder().getVelocity() /   (10.71) * Math.PI *  (0.1524) / 60
    );}




  



  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  

 
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }


  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }


public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    m_DifferentialDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getSpeeds() {
    return m_kinematics.toChassisSpeeds(wheelSpeeds);
  }










 
    }