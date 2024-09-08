package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTest;

public class DrivetrainTestConfig extends RobotConfig {

    public DrivetrainTest drivetrainTest;
    

    @Override
    public void init() {
        subsystems.clear();
        drivetrainTest = new DrivetrainTest();
        
    }
}