// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
//import edu.wpi.first.wpilibj.drive.MecanumDrive;


public class TeleopDriveCommand extends CommandBase {
  /** Creates a new TeleopDriveCommand. */
  public TeleopDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
    DriveSubsystem.leftBack.setInverted(true);
    DriveSubsystem.leftFront.setInverted(true);
    DriveSubsystem.robotDrive.setSafetyEnabled(false);
   /* if ((RobotContainer.wheelsGoBrr.getLeftY() <= 1) && (RobotContainer.wheelsGoBrr.getLeftY()>= 0) {
      DriveSubsystem.robotDrive.driveCartesian(0, 0, 0.1);
    } */
    // TESTING Direction of Wheels
    //DriveSubsystem.robotDrive.driveCartesian(0.1, -0.0, 0.0);
    //Negative Z turns robot clockwise!
    //Negative X reverses robot!
    //Positive X moves forward!

    //TESTING Direction of DROP Wheels
    DriveSubsystem.dropWheelDrive.arcadeDrive(0.1, 0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //DriveSubsystem.leftBack.setInverted(true);
    //DriveSubsystem.leftFront.setInverted(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    boolean isMecanum = true;
    if (isMecanum) {
      DriveSubsystem.robotDrive.driveCartesian(RobotContainer.wheelsGoBrr.getLeftY() * 0.3, RobotContainer.wheelsGoBrr.getLeftX() * 0.3, RobotContainer.wheelsGoBrr.getRightX() * 0.3);
    }
    else {
      DriveSubsystem.dropWheelDrive.arcadeDrive(RobotContainer.wheelsGoBrr.getLeftY() * 0.3, RobotContainer.wheelsGoBrr.getLeftX() * -0.3);
    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
