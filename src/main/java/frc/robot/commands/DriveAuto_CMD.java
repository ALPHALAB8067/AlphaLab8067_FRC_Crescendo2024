// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable_SS;

public class DriveAuto_CMD extends Command {
  /** Creates a new DriveWithJoystick. */

  private final BasePilotable_SS m_BasePilotable_SS;


  public DriveAuto_CMD(BasePilotable_SS p_BasePilotable_SS) {
      m_BasePilotable_SS = p_BasePilotable_SS;

      addRequirements(m_BasePilotable_SS); 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_BasePilotable_SS.tankDrive(1.0, 1.0);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_BasePilotable_SS.basePilotableStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
