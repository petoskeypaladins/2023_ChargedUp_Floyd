// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DropWheelsSubsystem extends SubsystemBase {
  /** Creates a new DropWheelsSubsystem. */
  public static CANSparkMax leftDrop = new CANSparkMax(Constants.LEFT_DROP, MotorType.kBrushless);
  public static CANSparkMax rightDrop = new CANSparkMax(Constants.RIGHT_DROP, MotorType.kBrushless);

  public static DifferentialDrive dropWheelDrive = new DifferentialDrive(rightDrop, leftDrop);

  public DropWheelsSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
