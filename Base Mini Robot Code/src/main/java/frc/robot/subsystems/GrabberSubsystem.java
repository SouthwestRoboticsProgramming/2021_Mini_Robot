/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class GrabberSubsystem extends SubsystemBase {
  private Solenoid grabberJaw = new Solenoid(Constants.PCMID, Constants.grabberJawSolenoidPort);
  private Servo grabberLifter = new Servo(Constants.grabberLiftPort);
  private DigitalInput grabberSwitch = new DigitalInput(0);

  private boolean lifted;
  private boolean grabbed;
  private boolean canHeld = false;

  public GrabberSubsystem() {
    setJawPosition(false);
    setLifterPosition(false);
  }

  public void setJawPosition(boolean grab) {
    this.grabbed = grab;
    grabberJaw.set(!grab);
    Robot.shuffleBoard.jawSolenoidPosition.setBoolean(grab);
  }

  public boolean isGrabbed() {
    return grabbed;
  }

  public void setLifterPosition(boolean lifted) {
    this.lifted = lifted;
    double angle;
    if (lifted) {
      angle = Robot.shuffleBoard.grabberLiftAngle.getDouble(0);
    } else {
      angle = Robot.shuffleBoard.grabberLowerAngle.getDouble(0);
    }
    grabberLifter.setAngle(angle);
    Robot.shuffleBoard.lifterServoAngle.setDouble(angle);
  }

  public boolean isLifted() {
    return lifted;
  }

  public boolean isGrabberSwitchPressed() {
    return !grabberSwitch.get();
  }

  @Override
  public void periodic() {
    if (isGrabberSwitchPressed() && !isGrabbed() && !canHeld) {
      setJawPosition(true);
      canHeld = true;
    }
    if (!isGrabberSwitchPressed()) {
      canHeld = false;
    }
  }
}
