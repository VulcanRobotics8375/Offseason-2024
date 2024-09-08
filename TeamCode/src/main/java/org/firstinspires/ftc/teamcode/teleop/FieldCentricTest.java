package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.utils.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.LVMaxbotixEZ4;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainRR;

import java.util.ArrayList;

@TeleOp
public class FieldCentricTest extends OpModePipeline {

    private final MainConfig subsystems = new MainConfig();
    DrivetrainRR drive;

    SimplePID turnPid = new SimplePID(-1.2, -0.01, 0.0, -0.3, 0.3);
    private LVMaxbotixEZ4 sensor1, sensor2, sensor3;

//    private KalmanFilter sensor2Filter = new KalmanFilter(0.6, 0.0001, 0.0, 0.1);
//    private KalmanFilter sensor3Filter = new KalmanFilter(0.6, 0.0001, 0.0, 0.1);

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double prevTime = timer.time();
    private ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

    public void init() {
//        sensor1 = hardwareMap.get(LVMaxbotixEZ4.class, "sensor_1");
//        sensor2 = hardwareMap.get(LVMaxbotixEZ4.class, "sensor_2");
//        sensor3 = hardwareMap.get(LVMaxbotixEZ4.class, "sensor_3");

//        sensor2.PRE_SCALAR_INCHES = 24.0 / 0.177;
//        sensor2.VOLTAGE_OFFSET = 2.271;
//
//        sensor3.PRE_SCALAR_INCHES = 24.0 / 0.414;
//        sensor3.VOLTAGE_OFFSET = 0.572;
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();

        drive = new DrivetrainRR(hardwareMap);
        subsystems.odoLift.liftOdoDown();
    }


    @Override
    public void loop() {
        Robot.update();
        Pose2d robotPose = drive.getPoseEstimate();

        if(timer.time() >= prevTime + 1000) {
            poses.add(robotPose);
        }

        double error = MathUtils.calcAngularError(0.0, robotPose.getHeading());


        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-robotPose.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        drive.update();
;//        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        subsystems.drivetrain.setPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad2.left_stick_y, -gamepad2.right_stick_y);

//        double robotTip = subsystems.drivetrain.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//        telemetry.addData("robot tip", robotTip);

//        sensor2Filter.run(sensor2.getDistance() - 4.25);
//        sensor3Filter.run(sensor3.getDistance());
//        double d1 = sensor2Filter.getEstimate();
//        double d2 = sensor3Filter.getEstimate();
//
//        double theta = MathUtils.fullAngleWrap(Math.atan((d1 - d2) / 8.0));

//        telemetry.addData("left", Robot.getConfiguration().localizer.getWheelPositions().get(0));
//        telemetry.addData("right", Robot.getConfiguration().localizer.getWheelPositions().get(1));
//        telemetry.addData("strafe", Robot.getConfiguration().localizer.getWheelPositions().get(2));
//        telemetry.addData("error", error);
//        telemetry.addData("output", output);
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
//        telemetry.addData("range heading", theta);

    }
}
