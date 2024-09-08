package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.EmptyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp
public class ServoTest extends OpModePipeline {
    public Servo servo;
    private double servoPos = 0.5;

    public void init() {
        runMode = RobotRunMode.TELEOP;
        subsystems = new EmptyConfig();
        super.init();

        servo = hardwareMap.servo.get("arm_left");
    }


    @Override
    public void loop() {
        Robot.update();
        servoPos += gamepad1.left_stick_y * 0.005;
        servo.setPosition(servoPos);
        telemetry.addData("servo pos", servoPos);
        telemetry.update();
    }
}