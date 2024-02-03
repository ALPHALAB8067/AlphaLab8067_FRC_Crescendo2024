// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable_SS;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Drive_Auto_CMD extends SequentialCommandGroup {
  /** Creates a new Drive_Auto_CMD. */



  public Drive_Auto_CMD( BasePilotable_SS m_BasePilotable_SS) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveAuto_CMD(m_BasePilotable_SS).andThen(new DriveAuto_CMD(m_BasePilotable_SS)));
  }
}
