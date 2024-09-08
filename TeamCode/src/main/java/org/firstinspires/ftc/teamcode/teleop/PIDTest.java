package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.EmptyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@Config
@TeleOp
public class PIDTest extends OpModePipeline {
    private DcMotorEx right, left;
    private Servo armLeft, armRight;

    private PIDController controller;

    public static double p = 0.0, i = 0.0, d = 0.0; //p = 0.024, i = 0, d = 0.0003;
    public static double f = 0.0; //0.19;

    public static int target = 0;

    public void init(){
        runMode = RobotRunMode.TELEOP;
        subsystems = new EmptyConfig();
        super.init();
        controller = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        right = hardwareMap.get(DcMotorEx.class, "lift_right");
        left = hardwareMap.get(DcMotorEx.class, "lift_left");
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.REVERSE);


        armLeft = hardwareMap.servo.get("arm_left");
        armRight = hardwareMap.servo.get("arm_right");
    }


    @Override
    public void loop() {
        Robot.update();

        controller.setPID(p, i, d);

        int pos = right.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = f;

        double power = pid + ff;

        right.setPower(power);
        left.setPower(power);

        telemetry.addData("pos", pos);
        telemetry.addData("target", target);
        telemetry.addData("pid+ff", pid+ff);

        setArmPositions(1.167);

//        telemetry.addData("current velocity", currentVelocity);
//        telemetry.addData("maximum velocity", maxVelocity);
//        telemetry.addData("left pos", left.getCurrentPosition());
//        telemetry.addData("right pos", right.getCurrentPosition());
//        telemetry.addData("left vel", left.getVelocity());
//        telemetry.addData("right vel", right.getVelocity());
//        telemetry.addData("target pos tolerance", right.getTargetPositionTolerance());
        telemetry.update();
    }

    public void setArmPositions(double encoderPos){
        armLeft.setPosition(getArmLeftPos(encoderPos));
        armRight.setPosition(getArmRightPos(encoderPos));
    }

    public double getArmLeftPos(double encoderPos) {
        return -0.38815791 * encoderPos + 1.0415263310180114;
    }

    private double getArmRightPos(double encoderPos) {
        return 0.41444162 * encoderPos + -0.06125086946586067;
    }
}
