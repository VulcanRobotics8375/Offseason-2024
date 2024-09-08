package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.EmptyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "PositionsTest")
public class PositionsTest extends OpModePipeline {

    EmptyConfig subsystems = new EmptyConfig();
    Servo armLeft, armRight;
    Servo claw;
    Servo odoLiftLeft, odoLiftMid;
    private AnalogInput armEncoder;
    private DistanceSensor coneSensor;
    private DistanceSensor distanceFront;

    private double leftPos = 0.582;
    private double rightPos = 0.567;
    private double encoderPos = 1.168;
//    private ArrayList<Double> rightPoses = new ArrayList<Double>();
//    private ArrayList<Double> leftPoses = new ArrayList<Double>();
//    private ArrayList<Double> encoderPoses = new ArrayList<Double>();
//    DecimalFormat df = new DecimalFormat("#.###");
//    private boolean updateButton = false;


    private boolean clawButton = false;
    private double clawOpen = 0.4758;
    private double clawClose = 0.5846;
    private boolean clawOpened = true;


    private double odoLeftPos = 0.99;
    private double odoMidPos = 0.06;

    DcMotorEx lift;
    private boolean hold = false;
    private int holdPosition;
    SimplePID liftPID = new SimplePID(0.003, 0.0, 0.0, -1.0, 1.0);

    public void init() {
        super.subsystems = subsystems;
        armLeft = hardwareMap.servo.get("arm_left");
        armRight = hardwareMap.servo.get("arm_right");
        claw = hardwareMap.servo.get("claw");
        odoLiftLeft = hardwareMap.servo.get("odo_lift_left");
        odoLiftMid = hardwareMap.servo.get("odo_lift_mid");

        armEncoder = hardwareMap.get(AnalogInput.class, "arm_encoder");
        encoderPos = getArmEncoderPos();

        coneSensor = hardwareMap.get(DistanceSensor.class, "cone_sensor");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distance_front");
//        distanceLeft = hardwareMap.get(DistanceSensor.class, "distance_left");
//        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit_switch");

//        limitSwitch.setMode();

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void start() {
        super.start();
    }

    public void loop() {
        Robot.update();

        telemetry.addData("distance front", distanceFront.getDistance(DistanceUnit.INCH));
//        telemetry.addData("distance left", distanceLeft.getDistance(DistanceUnit.INCH));

        encoderPos = Range.clip(encoderPos - gamepad2.right_stick_y*0.03,0.79, 3.0789);
        setArmPositions(encoderPos);

//        leftPos = leftPos + gamepad2.left_stick_y*0.005;
//
//        if (leftPos < 0.01) {
//            leftPos = 0.01;
//        }
//        else if(leftPos > 0.99){
//            leftPos = .99;
//        }
//        armLeft.setPosition(leftPos);
////////
//        rightPos = rightPos + gamepad2.right_stick_y*0.005;
//
//        if (rightPos < 0.01) {
//            rightPos = 0.01;
//        }
//        else if(rightPos > 0.99){
//            rightPos = .99;
//        }
//        armRight.setPosition(rightPos);




        telemetry.addData("leftPos", armLeft.getPosition());
        telemetry.addData("rightPos", armRight.getPosition());

        telemetry.addData("encoderPos", encoderPos);
        telemetry.addData("encoderVoltage", getArmEncoderPos());
        telemetry.addData("coneDist", coneSensor.getDistance(DistanceUnit.MM));

        if(gamepad2.a && !clawButton) {
            clawButton = true;
            clawOpened = !clawOpened;
        } else if(!gamepad2.a && clawButton) {
            clawButton = false;
        }
        claw.setPosition(clawOpened ? clawOpen : clawClose);
        telemetry.addData("clawOpened", clawOpened);

//        odoMidPos = Range.clip(odoMidPos + ((gamepad1.left_trigger > 0) ? -gamepad1.left_trigger : gamepad1.right_trigger)*0.01, 0.01, 0.99);
//        odoLiftMid.setPosition(odoMidPos);
//        odoLeftPos = Range.clip(odoLeftPos + ((gamepad2.left_trigger > 0) ? -gamepad2.left_trigger : gamepad2.right_trigger)*0.01, 0.01, 0.99);
//        odoLiftLeft.setPosition(odoLeftPos);
//        telemetry.addData("odoMidPos", odoMidPos);
//        telemetry.addData("odoLeftPos", odoLeftPos);
//
        holdPosition = Range.clip(holdPosition - (int)(gamepad2.left_stick_y*10), 0, 850);
        liftToPosition(holdPosition, 1.0);
        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("right current", lift.getCurrent(CurrentUnit.AMPS));
//
//        if(gamepad2.a && !updateButton) {
//            updateButton = true;
//            rightPoses.add(Double.parseDouble(df.format(rightPos)));
//            leftPoses.add(Double.parseDouble(df.format(leftPos)));
//            encoderPoses.add(Double.parseDouble(df.format(getArmEncoderPos())));
//        } else if(!gamepad2.a && updateButton) {
//            updateButton = false;
//        }
//        telemetry.addData("leftPoses", leftPoses.toString());
//        telemetry.addData("rightPoses", rightPoses.toString());
//        telemetry.addData("encoderPoses", encoderPoses.toString());

        telemetry.update();
    }

    public double getArmEncoderPos() {
        double voltage = armEncoder.getVoltage();
        return voltage;
    }

    public void setArmPositions(double encoderPos){
        armLeft.setPosition(getArmLeftPos(encoderPos));
        armRight.setPosition(getArmRightPos(encoderPos));
    }

    public double getArmLeftPos(double encoderPos) {
        return (0.33284293) * encoderPos + (-0.037280133182602704);
    }

    private double getArmRightPos(double encoderPos) {
        return (0.33742081) * encoderPos + (-0.05367464933032362);
    }

    public void liftToPosition(int pos, double power) {
        lift.setTargetPosition(pos);
        if(lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        lift.setPower(power);
    }

    public void liftManual(double liftStick) {
        int liftPos = lift.getCurrentPosition();
        double outputPower;

        if (Math.abs(liftStick) > 0.02) {
            liftStick *= -1;
            if (hold) {
                hold = false;
                liftPID.reset();
            }
            if (liftStick > 0) {
                if(liftPos > 800 - 300) {
                    outputPower = liftStick - ((liftStick / 300) * (liftPos - (800 - 300)));
                } else {
                    outputPower = liftStick;
                }
            } else {
                if(liftPos < 0 + 300) {
                    outputPower = (liftStick / 300) * (liftPos - 0);
                } else {
                    outputPower = liftStick;
                }
            }
        } else {
            if (!hold) {
                holdPosition = Range.clip(liftPos, 0, 800);
                hold = true;
            }
            outputPower = liftPID.run(holdPosition, liftPos);
        }

        telemetry.addData("lift power", outputPower);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("lift holding", hold);
        if(lift.getCurrent(CurrentUnit.AMPS) > 14 ){
            outputPower *= 0.8;
        }
        lift.setPower(outputPower);
    }
}