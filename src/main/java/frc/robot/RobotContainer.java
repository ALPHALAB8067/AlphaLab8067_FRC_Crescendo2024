// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveAuto_CMD;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.Drive_Auto_CMD;
import frc.robot.subsystems.BasePilotable_SS;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_XboxController = new XboxController(Constants.BPConstants.kDriverControllerPort);
private final BasePilotable_SS  m_BasePilotable_SS = new BasePilotable_SS();
  private final DriveWithJoystick m_DriveWithJoystick = new DriveWithJoystick(m_BasePilotable_SS, m_XboxController ); 
  private final DriveAuto_CMD  m_DriveAuto_CMD = new DriveAuto_CMD(m_BasePilotable_SS);
   private final Drive_Auto_CMD  m_Drive_Auto_CMD = new Drive_Auto_CMD(m_BasePilotable_SS);

private SendableChooser<Command> autoChooser  = new SendableChooser<>();
//SendableChooser<Command> m_autonomouschooser = new SendableChooser<>();





  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();

//autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
  // SmartDashboard.putData("Auto Mode", autoChooser);

  //Shuffleboard.getTab("Auto Chooser").add(m_autonomouschooser);
   // m_autonomouschooser.setDefaultOption("Auto 1", m_Drive_Auto_CMD  /*  new PathPlannerAuto("GroupedPathsAuto")*/);


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


   private void configureDefaultCommands() {
    m_BasePilotable_SS.setDefaultCommand(m_DriveWithJoystick);
 

  }



  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  /*    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());*/
    //SmartDashboard.putData( "PathPlanner", m_Drive_Auto_CMD  /*new PathPlannerAuto("GroupedPathsAuto")*/);

    SmartDashboard.putData("Auto Path", new PathPlannerAuto("AutoGroupedPath"));


  }


  public BasePilotable_SS getRobotDrive() {
    return m_BasePilotable_SS;
  }






  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   // return Autos.exampleAuto(m_exampleSubsystem);
    //return m_Drive_Auto_CMD;
    /// return new PathPlannerAuto("AutoGroupedPath");
     return autoChooser.getSelected();


   
  }
}
