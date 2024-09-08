package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift extends Subsystem {

    private DcMotorEx lift;
    private Servo armLeft, armRight;
    private Servo claw;
    private AnalogInput armEncoder;
    private ColorSensor coneSensor;

    private boolean hold = false;
    private int holdPosition;
    private double encoderPos;

    private final double STICK_THRESHOLD = 0.02;

    SimplePID liftPID = new SimplePID(0.003, 0., 0.0, -1.0, 1.0);
    private final int LIMIT_RANGE = 300;

    public final int MIN_POS = 0;
    public int LIFT_INTAKE = 0;
    public int[] LIFT_INTAKE_STACK = {146, 183, 231, 287, 330}; // top place 268, increment 53
    public int LIFT_INTAKE_STACK_DROP = 30;
    public int LIFT_WALL_CLEAR = 580;

    public final int TERMINAL_LEVEL = 0;
    public final int GROUND_LEVEL = 0;
    public final int LOW_LEVEL = 10;
    public final int LOW_PLACE = 0;
    public final int MED_LEVEL = 445;
    public final int MED_PLACE = 0;
    public final int HIGH_LEVEL = 824;
    public final int HIGH_PLACE = 0;
    public final int MAX_POS = 850;

    private boolean liftIncDown = false;
    private boolean liftIncUp = false;
    private int liftIncrement = 5;

    private boolean armMidButton = false;
    private boolean inArmMid = false;

    public final double ARM_INTAKE = 4.151;
    public final double ARM_INTAKE_CLEAR = 4.279;
    public double ARM_INTAKE_STACK = 4.304;
    public final double ARM_HIGH_DEPOSIT = 2.88;
    public final double ARM_MED_DEPOSIT = 2.858;
    public final double ARM_LOW_DEPOSIT = 2.881;
    public final double ARM_TERMINAL_DEPOSIT = 2.546;
    public final double ARM_GROUND_DEPOSIT = 4.082;
    public final double ARM_MID = 3.156;
    public final double ARM_HORIZ = 3.952;
    public final double ARM_18 = 3.625;
    public final double ARM_MIN = 2.546;
    public final double ARM_MAX = 4.3695;

    private boolean clawButton = false;
    private boolean clawOpen = true;

    private final double CLAW_OPEN_DEPOSIT = 0.4758;
    private final double CLAW_OPEN_INTAKE = 0.4758;
    private final double CLAW_OPEN_INTAKE_AUTO = 0.277;
    private final double CLAW_CLOSED_POS = 0.5846;
    private double clawPos;
    private ElapsedTime clawCloseTimer = new ElapsedTime();
    private ElapsedTime clawOpenTimer = new ElapsedTime();
    private ElapsedTime grabTimer = new ElapsedTime();

    private boolean liftButton = false;
    private boolean runningToLevel = true;
    private int currentLevel = 0;

    private boolean wallCleared = false;
    private boolean coneCleared = false;

    private boolean switchIntake = false;

    private int coneStack = 4;
    private boolean liftStackUp = false;
    private boolean liftStackDown = false;

    private boolean firstLoop = true;
    private ElapsedTime colorTimer = new ElapsedTime();

    @Override
    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

//        lift.setVelocityPIDFCoefficients(1.3, 0.13, 0, 13.0);
//        lift.setPositionPIDFCoefficients(20.0);


//        right.setPositionPIDFCoefficients();

        armLeft = hardwareMap.servo.get("arm_left");
        armRight = hardwareMap.servo.get("arm_right");

        armEncoder = hardwareMap.get(AnalogInput.class, "arm_encoder");

        coneSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        claw = hardwareMap.servo.get("claw");
//        claw.setPosition(CLAW_CLOSED_POS);
    }

    //independent state machine variables
    //there is a main state machine, liftState, that controls most of the lift.
    // Some components of the lift are state-independent or need slightly more complex logic,
    // so we add some booleans to control those states.
    RobotState robotState = RobotState.INTAKE;
    RobotState prevRobotState = RobotState.INTAKE;
    RobotState intakeState = RobotState.INTAKE;
    boolean inIntakePosition;
    public void run(boolean terminal, boolean ground, boolean low, boolean med, boolean high, boolean clawButton, boolean armMidButton, boolean switchIntake, boolean home, double liftStick, double armStick, boolean liftStackUp, boolean liftStackDown, boolean liftIncDown, boolean liftIncUp, boolean liftResetButton) {
        if(firstLoop) {
            colorTimer.reset();
            firstLoop = false;
        }

        int liftPos = getLiftPosition();
        telemetry.addData("liftPos", liftPos);

        if(switchIntake && !this.switchIntake) {
            this.switchIntake = true;
            intakeState = (intakeState == RobotState.INTAKE) ? RobotState.INTAKE_STACK : RobotState.INTAKE;
            if(robotState == RobotState.INTAKE || robotState == RobotState.INTAKE_STACK) {
                robotState = intakeState;
            }
        } else if(!switchIntake && this.switchIntake) {
            this.switchIntake = false;
        }

        double armStackPos = (coneStack==0) ? ARM_INTAKE : ARM_INTAKE_STACK;

        if(liftStackUp && !this.liftStackUp) {
            this.liftStackUp = true;
            if(coneStack < 4) coneStack += 1;
        } else if(!liftStackUp && this.liftStackUp) {
            this.liftStackUp = false;
        }
        if(liftStackDown && !this.liftStackDown) {
            this.liftStackDown = true;
            if(coneStack > 0) coneStack -= 1;
        } else if(!liftStackDown && this.liftStackDown) {
            this.liftStackDown = false;
        }

        if(liftIncUp && !this.liftIncUp) {
            this.liftIncUp = true;
            LIFT_INTAKE += liftIncrement;
            for(int i = 0; i < LIFT_INTAKE_STACK.length; i++) {
                LIFT_INTAKE_STACK[i] += liftIncrement;
            }
        } else if(!liftIncUp && this.liftIncUp) {
            this.liftIncUp = false;
        }
        if(liftIncDown && !this.liftIncDown) {
            this.liftIncDown = true;
            LIFT_INTAKE -= liftIncrement;
            for(int i = 0; i < LIFT_INTAKE_STACK.length; i++) {
                LIFT_INTAKE_STACK[i] -= liftIncrement;
            }
        } else if(!liftIncDown && this.liftIncDown) {
            this.liftIncDown = false;
        }

        if(home || terminal || ground || low || med || high) {
            liftButton = true;
            if (home) {
                robotState = intakeState;
            } else if (terminal) {
                currentLevel = 0;
            } else if (ground) {
                currentLevel = 1;
            } else if (low) {
                currentLevel = 2;
            } else if (med) {
                currentLevel = 3;
            } else if (high) {
                currentLevel = 4;
            }
        } else {
            liftButton = false;
        }

        // Switch back to joystick
        if(runningToLevel && !liftButton && (Math.abs(liftStick) > STICK_THRESHOLD || Math.abs(armStick) > STICK_THRESHOLD)) {
            runningToLevel = false;
            robotState = RobotState.MANUAL;
        }

//        RobotState prevRobotState = robotState;
        switch (robotState) {
            case INTAKE:
                telemetry.addData("robotState", "INTAKE");

                wallCleared = true;
                coneCleared = false;

                if(prevRobotState == RobotState.DEPOSIT) {
                    clawOpenTimer.reset();
                }
                // switch mode to runningToLevel
                if(!runningToLevel) {
                    runningToLevel = true;
                }
                if(clawOpenTimer.milliseconds() > 100) {
                    if (liftPos > LIFT_INTAKE+100) {
                        telemetry.addData("liftState", "HIGH");
                        setArmPositions(ARM_MID);
                        if (Math.abs(getArmEncoderPos() - ARM_MID) < 0.1) {
                            liftToPosition(LIFT_INTAKE);
                        }
                    } else if (liftPos > LIFT_INTAKE - 20) {
                        telemetry.addData("liftState", "HOVER");
                        setArmPositions(ARM_INTAKE);
                        liftToPosition(LIFT_INTAKE);
                    } else {
                        telemetry.addData("liftState", "LOW");
                        setArmPositions(ARM_HORIZ);
                        liftToPosition(LIFT_INTAKE);
                    }
                }


                if(getArmEncoderPos() < ARM_HORIZ) {
                    clawPos = CLAW_OPEN_DEPOSIT;
                } else {
                    clawPos = CLAW_OPEN_INTAKE;
                }

                inIntakePosition = Math.abs(ARM_INTAKE - getArmEncoderPos()) < 0.2;

                if(clawButton && !this.clawButton && inIntakePosition){
                    this.clawButton = true;
                    clawOpen = false;
                    clawPos = CLAW_CLOSED_POS;
                    robotState = RobotState.DEPOSIT;
                    prevRobotState = RobotState.INTAKE;
                    break;
                } else if(!clawButton && this.clawButton) {
                    this.clawButton = false;
                }

//                if (colorTimer.seconds() > 1 && (coneSensor.red() > 250 || coneSensor.blue() > 110) && inIntakePosition){
//                    clawOpen = false;
//                    clawPos = CLAW_CLOSED_POS;
//                    robotState = RobotState.DEPOSIT;
//                    prevRobotState = RobotState.INTAKE;
//                    break;
//                }

                claw.setPosition(clawPos);

                prevRobotState = RobotState.INTAKE;
                break;

            case INTAKE_STACK:
                telemetry.addData("robotState", "INTAKE_STACK");

                wallCleared = false;
                coneCleared = true;

                if(prevRobotState == RobotState.DEPOSIT) {
                    clawOpenTimer.reset();
                }
                // switch mode to runningToLevel
                if(!runningToLevel) {
                    runningToLevel = true;
                }
                if(clawOpenTimer.milliseconds() > 100) {
                    if (liftPos > LIFT_INTAKE_STACK[coneStack]+100) {
                        telemetry.addData("liftState", "HIGH");
                        setArmPositions(ARM_MID);
                        if (Math.abs(getArmEncoderPos() - ARM_MID) < 0.1) {
                            liftToPosition(LIFT_INTAKE_STACK[coneStack] + LIFT_INTAKE_STACK_DROP);
                        }
                    } else if (liftPos > LIFT_INTAKE) {
                        telemetry.addData("liftState", "HOVER");
                        setArmPositions(armStackPos);
                        liftToPosition(LIFT_INTAKE_STACK[coneStack] + LIFT_INTAKE_STACK_DROP);
                    } else {
                        telemetry.addData("liftState", "LOW");
                        setArmPositions(ARM_HORIZ);
                        liftToPosition(LIFT_INTAKE_STACK[coneStack] + LIFT_INTAKE_STACK_DROP);
                    }
                }


                if(getArmEncoderPos() < ARM_HORIZ) {
                    clawPos = CLAW_OPEN_DEPOSIT;
                } else {
                    clawPos = CLAW_OPEN_INTAKE;
                }

//                inIntakePosition = (Math.abs(ARM_INTAKE_STACK - getArmEncoderPos()) < 0.2) && (Math.abs(liftPos - LIFT_HOVER_STACK) < 20);
                inIntakePosition = Math.abs((armStackPos) - getArmEncoderPos()) < 0.2;

                if(clawButton && !this.clawButton && inIntakePosition){
                    this.clawButton = true;
                    clawOpen = false;
                    clawPos = CLAW_CLOSED_POS;
                    robotState = RobotState.GRAB_STACK;
                    prevRobotState = RobotState.INTAKE_STACK;
                    grabTimer.reset();
                    break;
                } else if(!clawButton && this.clawButton) {
                    this.clawButton = false;
                }

//                if (colorTimer.seconds() > 1 && (coneSensor.red() > 270 || coneSensor.blue() > 140) && inIntakePosition && (Math.abs(liftPos - LIFT_HOVER) < 20)){
//                    clawOpen = false;
//                    clawPos = CLAW_CLOSED_POS;
//                    robotState = RobotState.GRAB_STACK;
//                    prevRobotState = RobotState.INTAKE_STACK;
//                    grabTimer.reset();
//                    break;
//                }

                claw.setPosition(clawPos);

                prevRobotState = RobotState.INTAKE_STACK;
                break;

            case GRAB_STACK:
                telemetry.addData("robotState", "GRAB_STACK");

                if(prevRobotState != RobotState.GRAB_STACK) {
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                setPower(0.05);
                setArmPositions(armStackPos);
                if(grabTimer.milliseconds() > 100) {
                    clawOpen = false;
                    clawPos = CLAW_CLOSED_POS;
                    robotState = RobotState.DEPOSIT;
                }
                prevRobotState = RobotState.GRAB_STACK;
                break;

            case DEPOSIT:
                if(prevRobotState == RobotState.INTAKE || prevRobotState == RobotState.GRAB_STACK) {
                    clawCloseTimer.reset();
                }
                if(prevRobotState == RobotState.GRAB_STACK) {
                    if(coneStack > 0) coneStack -= 1;
                }
                if(prevRobotState != RobotState.DEPOSIT) {
                    inArmMid = false;
                }
                if(clawCloseTimer.milliseconds() > 225) {
                    if(!clawOpen) {
                        if(wallCleared) {
                            if (currentLevel == 0) {
                                liftToPosition(TERMINAL_LEVEL, 0.5);
                            } else if (currentLevel == 1) {
                                liftToPosition(GROUND_LEVEL, 0.5);
                            } else if (currentLevel == 2) {
                                liftToPosition(LOW_LEVEL);
                            } else if (currentLevel == 3) {
                                liftToPosition(MED_LEVEL);
                            } else if (currentLevel == 4) {
                                liftToPosition(HIGH_LEVEL);
                            }
                            if(!inArmMid) {
                                if (currentLevel == 0) {
                                    setArmPositions(ARM_TERMINAL_DEPOSIT);
                                } else if (currentLevel == 1) {
                                    setArmPositions(ARM_GROUND_DEPOSIT);
                                } else if (currentLevel == 2) {
                                    setArmPositions(ARM_LOW_DEPOSIT);
                                } else if (currentLevel == 3) {
                                    setArmPositions(ARM_MED_DEPOSIT);
                                } else if (currentLevel == 4) {
                                    setArmPositions(ARM_HIGH_DEPOSIT);
                                }
                            } else {
                                setArmPositions(ARM_MID);
                            }
                        } else {
                            if(coneCleared) {
                                liftToPosition(LIFT_WALL_CLEAR);
                                if (Math.abs(liftPos - LIFT_WALL_CLEAR) < 30) {
                                    wallCleared = true;
                                }
                            } else {
                                setArmPositions(ARM_INTAKE_CLEAR);
                                if(getArmEncoderPos() < ARM_INTAKE_CLEAR+0.02) {
                                    coneCleared = true;
                                }
                            }
                        }
                    }
                }

                if(armMidButton && !this.armMidButton) {
                    this.armMidButton = true;
                    inArmMid = !inArmMid;
                } else if(!armMidButton && this.armMidButton) {
                    this.armMidButton = false;
                }

                if(clawButton && !this.clawButton && !inArmMid){
                    this.clawButton = true;
                    clawOpen = true;
                    clawPos = CLAW_OPEN_DEPOSIT;
                    robotState = intakeState;
                    prevRobotState = RobotState.DEPOSIT;
                    break;
                } else if(!clawButton && this.clawButton) {
                    this.clawButton = false;
                }

                claw.setPosition(clawPos);

                prevRobotState = RobotState.DEPOSIT;
                break;

            case MANUAL:
                if(prevRobotState != RobotState.MANUAL) {
                    holdPosition = liftPos;
                    encoderPos = getArmEncoderPos();
                }
                // run joystick
                holdPosition = holdPosition + (int)(liftStick*10);
                liftToPosition(holdPosition);

                encoderPos = Range.clip(encoderPos + armStick*0.03, ARM_MIN, ARM_MAX);
                setArmPositions(encoderPos);

                if (clawButton && !this.clawButton) {
                    this.clawButton = true;
                    clawOpen = !clawOpen;
                } else if (!clawButton && this.clawButton) {
                    this.clawButton = false;
                }

                if(clawOpen) {
                    clawPos = (getArmEncoderPos() < ARM_HORIZ/2 + ARM_MID/2) ?  CLAW_OPEN_DEPOSIT : CLAW_OPEN_INTAKE;
                } else {
                    clawPos = CLAW_CLOSED_POS;
                }

                claw.setPosition(clawPos);

                telemetry.addData("lift pos", liftPos);
                telemetry.addData("arm target pos", encoderPos);
                telemetry.addData("claw open", clawOpen);

                if(liftResetButton) {
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    holdPosition = 0;
                }

                prevRobotState = RobotState.MANUAL;
                break;
        }

    }

    public void setMode(DcMotor.RunMode mode) {
        lift.setMode(mode);
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    enum RobotState {
        INTAKE,
        INTAKE_STACK,
        GRAB_STACK,
        DEPOSIT,
        MANUAL
    }

    public void liftToPosition(int pos, double power) {
        lift.setTargetPosition(pos);
        if(lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        setPower(power);
    }

    public void liftToPosition(int pos) {
        liftToPosition(pos, 1.0);
    }

    public double getArmEncoderPos() {
        double voltage = armEncoder.getVoltage();
        return (voltage < 2) ? voltage + 3.25 : voltage;
    }

    public void setArmPositions(double encoderPos){
        armLeft.setPosition(getArmLeftPos(encoderPos));
        armRight.setPosition(getArmRightPos(encoderPos));
    }

    public double getArmLeftPos(double encoderPos) {
        return (-0.43559021) * encoderPos + (2.0937054147375633);
    }

    private double getArmRightPos(double encoderPos) {
        return (0.42194254) * encoderPos + (-0.9839818034416186);
    }

    public int getLiftPosition() {
        return lift.getCurrentPosition();
    }

    public void clawClose(){
        claw.setPosition(CLAW_CLOSED_POS);
    }
    public void clawOpenIntake() {
        claw.setPosition(CLAW_OPEN_INTAKE);
    }
    public void clawOpenIntakeAuto() { claw.setPosition(CLAW_OPEN_INTAKE_AUTO); }
    public void clawOpenDeposit() {
        claw.setPosition(CLAW_OPEN_DEPOSIT);
    }

    public void clawSetPosition(double clawPos){
        claw.setPosition(clawPos);
    }

    public void deadLift() {
        telemetry.addData("liftRightPos", lift.getCurrentPosition());
        telemetry.addData("clawColorValue - Red", coneSensor.red());
        telemetry.addData("clawColorValue - Blue", coneSensor.blue());
        telemetry.addData("clawColorValue - Green", coneSensor.green());
    }

    public void stop() {
        setArmPositions(ARM_MID);
        claw.setPosition(CLAW_CLOSED_POS);
        while(Math.abs(getArmEncoderPos() - ARM_MID) > 0.5) {}
        super.stop();
    }
}