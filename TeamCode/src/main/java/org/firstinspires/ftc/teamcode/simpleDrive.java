package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/* TODO:
*   1. test the code
*   2. add PID? or odometry
* */

@TeleOp(name = "simpleDrive")
public class simpleDrive extends OpMode {
    // declaration
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    BNO055IMU imu; // imu embedded into the control hub

    @Override
    public void init(){
        // initialization
        fl = hardwareMap.get(DcMotor.class,"front_left");
        fr = hardwareMap.get(DcMotor.class,"front_right");
        bl = hardwareMap.get(DcMotor.class,"back_left");
        br = hardwareMap.get(DcMotor.class,"back_right");

        // run w/o using encoder; we're controlling it with a joystick + don't need
        // constant velocity to be maintained
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // left side motors = reversed, right side = forward
        // this way, they'll both work in the same direction
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /* NOTE: up is negative for y stick + down is positive
    *  both sticks range from -1 to 1
    *  left stick's x axis = controls direction
    *  right stick's y axis = controls power
    * */
    public void loop(){
        // movement code
        double y = -gamepad1.left_stick_y; // sets power
        double x = gamepad1.left_stick_x; // strafing
        double rx = gamepad1.right_stick_x; // turning

        // scales it down if any of the values are above 1.00
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        // setting powers
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        // testing to see what's wrong?
        telemetry.addData("front left power", flPower);
        telemetry.addData("front right power", frPower);
        telemetry.addData("back left power", blPower);
        telemetry.addData("back right power", brPower);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
