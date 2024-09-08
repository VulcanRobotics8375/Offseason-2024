package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainRR;

@TeleOp(name = "ADrive")
public class ADrive extends OpModePipeline {

    DrivetrainConfig subsystems = new DrivetrainConfig();

    Drivetrain drive;

    public void init() {
        super.subsystems = subsystems;

        runMode = RobotRunMode.TELEOP;
        super.init();

        drive = new Drivetrain();
    }

    public void start() {
        super.start();
    }

    public void loop() {
        Robot.update();

        drive.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

//        telemetry.addData("x", robotPose.getX());
//        telemetry.addData("y", robotPose.getY());
//        telemetry.addData("heading", robotPose.getHeading());
//        telemetry.addData("speed", drive.getPoseVelocity().vec().norm());
//        telemetry.addData("encoder raw positions", Robot.localizer.getRawWheelPositions().toString());
        telemetry.update();
    }
}