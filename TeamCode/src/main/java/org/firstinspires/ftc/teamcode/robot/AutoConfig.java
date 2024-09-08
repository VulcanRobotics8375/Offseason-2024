package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftHover;
import org.firstinspires.ftc.teamcode.subsystems.OdoLift;

public class AutoConfig extends RobotConfig {

    public LiftHover lift;
    public OdoLift odoLift;
    public Drivetrain drivetrain;

    @Override
    public void init() {
        subsystems.clear();
        lift = new LiftHover();
        odoLift = new OdoLift();
        drivetrain = new Drivetrain();

        subsystems.add(lift);
        subsystems.add(odoLift);
        subsystems.add(drivetrain);
    }
}