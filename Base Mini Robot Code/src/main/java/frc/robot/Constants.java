/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //TODO: Possibly use convention of PORT_NAME_1 for finals?
  //TODO: Classify by what kind of ports used rather than type of thing?

  public static final int PCMID = 37;
  public static final int controllerPort = 0;

  //Motor controllers
  public static final int leftPort1 = 1,
                          rightPort1 = 2;

  //Solenoids
  public static final int grabberJawSolenoidPort = 0;

  //PWM
  public static final int grabberLiftPort = 0;

  public final double wheelCircumferenceMeters = 0.1524;
  public final int encoderTicksPerRotation = 4096;
}
