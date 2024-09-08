package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.DrivetrainTestConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(group = "example", name = "example")
public class ExampleOpMode extends OpModePipeline {
    DrivetrainTestConfig subsystems = new DrivetrainTestConfig();

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        Robot.update();
        subsystems.drivetrainTest.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.update();
    }
}
