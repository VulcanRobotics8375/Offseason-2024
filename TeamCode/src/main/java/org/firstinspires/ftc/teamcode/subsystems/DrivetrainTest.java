package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.robotcorelib.math.utils.MathUtils.joystickCurve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.JoystickCurve;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class DrivetrainTest extends Subsystem {
    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;

    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorBackLeft = hardwareMap.dcMotor.get("back_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double forward, double strafe, double turn) {


        double coefficient = (Math.abs(forward)+Math.abs(strafe))/Math.sqrt(Math.pow(forward, 2)+Math.pow(strafe, 2));

        //diagonalq
        motorFrontLeft.setPower((forward-strafe)/coefficient);
        motorBackLeft.setPower((forward+strafe)/coefficient);
        motorFrontRight.setPower((forward+strafe)/coefficient);
        motorBackRight.setPower((forward-strafe)/coefficient);
    }
    public void mechanumDrive(double forward, double strafe, double turn) {
        double multiplier = 2.0 / Math.sqrt(2.0);
        forward = joystickCurve(forward, JoystickCurve.MODIFIED_CUBIC);
        strafe = joystickCurve(strafe, JoystickCurve.MODIFIED_CUBIC);
        turn = joystickCurve(turn, JoystickCurve.MODIFIED_CUBIC);

        double theta = Math.atan2(forward, strafe) - Math.PI / 4.0;
//        turn = joystickCurve(turn, JoystickCurve.MODIFIED_CUBIC);

        double magnitude = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if(magnitude > 1) {
            forward *= 1 / magnitude;
            strafe *= 1 / magnitude;
            turn *= 1 / magnitude;
        }

        // Godly Math Trick: sin(x+pi/4) = cos(x-pi/4)
        double speed = multiplier * Math.hypot(strafe, forward);

        double flSpeed = speed * Math.cos(theta) + turn;
        double frSpeed = speed * Math.sin(theta) - turn;
        double blSpeed = speed * Math.sin(theta) + turn;
        double brSpeed = speed * Math.cos(theta) - turn;

        motorFrontLeft.setPower(flSpeed);
        motorFrontRight.setPower(frSpeed);
        motorBackLeft.setPower(blSpeed);
        motorBackRight.setPower(brSpeed);
    }
}