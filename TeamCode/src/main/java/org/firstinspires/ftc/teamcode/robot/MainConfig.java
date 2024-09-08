package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftHover;
import org.firstinspires.ftc.teamcode.subsystems.OdoLift;

public class MainConfig extends RobotConfig {

    public Drivetrain drivetrain;
    public LiftHover lift;
    public OdoLift odoLift;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
        lift = new LiftHover();
        odoLift = new OdoLift();
    }
}
