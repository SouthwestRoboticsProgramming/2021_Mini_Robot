package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Add your docs here.
 */
public class ShuffleBoard{

    public boolean update = false;
    public ShuffleboardTab GamePlay = Shuffleboard.getTab("GamePlay");
        public NetworkTableEntry path1 = GamePlay.add("path1", "").getEntry();
        public NetworkTableEntry path2 = GamePlay.add("path2", "").getEntry();
        public NetworkTableEntry path3 = GamePlay.add("path3", "").getEntry();
        public NetworkTableEntry path4 = GamePlay.add("path4", "").getEntry();
        
    public ShuffleboardTab Tune = Shuffleboard.getTab("Tune");
        private ShuffleboardLayout driveTune = Tune.getLayout("driveTune", BuiltInLayouts.kList);
            public NetworkTableEntry driveType = driveTune.addPersistent("driveType", "c").getEntry(); // a=arcade, c=cheezy, f=field
            public NetworkTableEntry driveSpeed = driveTune.addPersistent("driveSpeed", 0).getEntry();
            public NetworkTableEntry driveSmooth = driveTune.addPersistent("driveSmooth", 0).getEntry();
            public NetworkTableEntry driveFXPidP = driveTune.addPersistent("driveFXPID-P", 0).getEntry();
            public NetworkTableEntry driveFXPidI = driveTune.addPersistent("driveFXPID-I", 0).getEntry();
            public NetworkTableEntry driveFXPidD = driveTune.addPersistent("driveFXPID-D", 0).getEntry();
            public NetworkTableEntry driveFXPidF = driveTune.addPersistent("driveFXPID-F", 0).getEntry();
            public NetworkTableEntry driveTurnPidP = driveTune.addPersistent("driveTurnPID-P", 0).getEntry();
            public NetworkTableEntry driveTurnPidI = driveTune.addPersistent("driveTurnPID-I", 0).getEntry();
            public NetworkTableEntry driveTurnPidD = driveTune.addPersistent("driveTurnPID-D", 0).getEntry();
            public NetworkTableEntry driveTurnPidF = driveTune.addPersistent("driveTurnPID-F", 0).getEntry();
            public NetworkTableEntry driveArcadeSpeed = driveTune.addPersistent("driveArcadeSpeed", 0).getEntry();
            public NetworkTableEntry driveArcadeTurn = driveTune.addPersistent("driveArcadeTurn", 0).getEntry();

        private ShuffleboardLayout grabberTune = Tune.getLayout("grabberTune", BuiltInLayouts.kList);
            public NetworkTableEntry grabberLiftAngle = grabberTune.addPersistent("grabberLiftAngle", 0).getEntry();
            public NetworkTableEntry grabberLowerAngle = grabberTune.addPersistent("grabberLowerAngle", 0).getEntry();

    public ShuffleboardTab Debug = Shuffleboard.getTab("Debug");
        private ShuffleboardLayout driveDebug = Debug.getLayout("driveDebug", BuiltInLayouts.kList);
            public NetworkTableEntry driveLeftOutput = driveDebug.add("driveLeftOutput", 0).getEntry();
            public NetworkTableEntry driveRightOutput = driveDebug.add("driveRightOutput", 0).getEntry();
            public NetworkTableEntry driveControlMode = driveDebug.add("driveControlMode", 0).getEntry();
            public NetworkTableEntry driveCurrentType = driveDebug.add("driveCurrentType", "not set!").getEntry();
            public NetworkTableEntry drivePosition = driveDebug.add("drivePosition", "").getEntry();
        
        private ShuffleboardLayout grabberDebug = Debug.getLayout("grabberDebug", BuiltInLayouts.kList);
            public NetworkTableEntry jawSolenoidPosition = grabberDebug.add("jawSolenoidPosition", "null").getEntry();
            public NetworkTableEntry lifterServoAngle = grabberDebug.add("servoAngle", 0).getEntry();

    public ShuffleBoard() {

    }
}   
