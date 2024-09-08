package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainRR;

@TeleOp(name = "Main")
public class Main extends OpModePipeline {

    MainConfig subsystems = new MainConfig();
    DrivetrainRR drive;

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();

        drive = new DrivetrainRR(hardwareMap);
    }

    public void start() {
        super.start();
        subsystems.odoLift.liftOdoUp();
    }

    public void loop() {
        Robot.update();

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        joystickCurve(-gamepad1.right_stick_x)
                )
        );
        subsystems.lift.run(
                gamepad2.a || gamepad1.a,
                gamepad2.left_trigger>0 || gamepad1.left_trigger>0,
                gamepad2.b || gamepad1.b,
                gamepad2.x || gamepad1.x,
                gamepad2.y || gamepad1.y,
                gamepad2.left_bumper || gamepad1.left_bumper,
                gamepad2.dpad_right || gamepad1.dpad_right,
                gamepad1.touchpad || gamepad1.touchpad_finger_1 | gamepad1.touchpad_finger_2,
                gamepad2.right_bumper || gamepad1.right_bumper,
                gamepad2.right_trigger > 0 || gamepad1.right_trigger>0,
                -gamepad2.left_stick_y*1,
                -gamepad2.right_stick_y*1,
                gamepad2.dpad_up || gamepad1.dpad_up,
                gamepad2.dpad_down || gamepad1.dpad_down,
                false
        );
//        subsystems.lift.run(
//                gamepad2.a==true,
//                gamepad2.left_trigger>0,
//                gamepad2.b==true,
//                gamepad2.x==true,
//                gamepad2.y==true,
//                gamepad2.left_bumper || gamepad1.left_bumper,
//                gamepad2.dpad_right==true,
//                gamepad1.b==true,
//                gamepad2.right_bumper==true,
//                gamepad2.right_trigger > 0,
//                -gamepad2.left_stick_y,
//                -gamepad2.right_stick_y,
//                gamepad2.dpad_up==true,
//                gamepad2.dpad_down==true,
//                gamepad1.a==true
//        );

        drive.update();
        telemetry.update();
    }

    private double joystickCurve(double x) {
        return (x/1.07) * (0.62*Math.pow(x, 2) + 0.45);
    }


}
