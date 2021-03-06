/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ManualDriveCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;

  public ManualDriveCommand(DriveTrainSubsystem driveTrainSubsystem) {
    addRequirements(driveTrainSubsystem);
    this.m_driveTrainSubsystem = driveTrainSubsystem;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveSpeed = Robot.shuffleBoard.driveArcadeSpeed.getDouble(1);
    double driveTurn = Robot.shuffleBoard.driveArcadeTurn.getDouble(.5);
    double xLeft = -Robot.robotContainer.getLeftTurn();
    double yLeft = -Robot.robotContainer.getLeftDrive();
    m_driveTrainSubsystem.driveMotors((yLeft + xLeft)*.3, (yLeft - xLeft)*.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
