package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class LiftSimple extends Subsystem {
    private DcMotorEx lift;
    private Servo armLeft, armRight;
    private Servo claw;
    private AnalogInput armEncoder;
    private double encoderPos;
    private int targetPos = 0;
    private boolean liftButton1 = false;
    private boolean liftButton2 = false;
    private boolean liftButton3 = false;
    private boolean liftButton4 = false;
    private boolean clawButton = false;
    private boolean clawOpen = true;
    public static final double CLAW_OPEN = 0.7;
    public static final double CLAW_CLOSE = 0.3;

    //0.79, 3.0789; top secret num

    @Override
    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        armLeft = hardwareMap.get(Servo.class, "arm_left");
        armRight = hardwareMap.get(Servo.class,"arm_right");
        claw = hardwareMap.get(Servo.class,"claw");
        armEncoder = hardwareMap.get(AnalogInput.class,"arm_encoder");
        lift.setDirection(DcMotor.Direction.REVERSE);

    }

//    public void run(double liftStick){
//       double currentPos = lift.getCurrentPosition();
//
//        targetPos = Range.clip(targetPos + 10*liftStick, 0, 850);
//        liftToPosition((int) (targetPos), 1);
//
//        telemetry.addData("current pos", currentPos);
//        telemetry.addData("target pos", targetPos);
//    }



    public void run(boolean liftButton1, boolean liftButton2, boolean liftButton3, boolean liftButton4, double encoderPos, double armStick, boolean clawButton){
        double currentPos = lift.getCurrentPosition();
        if (liftButton1 && !this.liftButton1) {
            this.liftButton1 = true;
            targetPos = (100);
        } else if (!liftButton1 && this.liftButton1) {
            this.liftButton1 = false;
        }

        if (liftButton2 && !this.liftButton2) {
            this.liftButton2 = true;
            targetPos = (200);
        } else if (!liftButton2 && this.liftButton2) {
            this.liftButton2 = false;
        }

        if (liftButton3 && !this.liftButton3) {
            this.liftButton3 = true;
            targetPos = (300);
        } else if (!liftButton3 && this.liftButton3) {
            this.liftButton3 = false;
        }

        if (liftButton4 && !this.liftButton4) {
            this.liftButton4 = true;
            targetPos = (400);

        } else if (!liftButton4 && this.liftButton4) {
            this.liftButton4 = false;
        }

        liftToPosition((int) (targetPos), 1);

        encoderPos = Range.clip(encoderPos + (armStick * 0.01), 0.79, 3.0789);
        armLeft.setPosition(getArmLeftPos(encoderPos));
        armRight.setPosition(getArmRightPos(encoderPos));

        if (clawButton && !this.clawButton) {
            this.clawButton = true;
            clawOpen = !clawOpen;



        } else if (!clawButton && this.clawButton) {
            this.clawButton = false;
        }
        if (clawOpen) {
            claw.setPosition(CLAW_OPEN);
        } else {
            claw.setPosition(CLAW_CLOSE);
        }
    }



    public void liftToPosition(int pos, double power) {
        lift.setTargetPosition(pos);
        if(lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        lift.setPower(power);
    }

    public double getArmLeftPos(double encoderPos) {
        return (0.33284293) * encoderPos + (-0.037280133182602704);
    }



    private double getArmRightPos(double encoderPos) {
        return (0.33742081) * encoderPos + (-0.05367464933032362);
    }
}







