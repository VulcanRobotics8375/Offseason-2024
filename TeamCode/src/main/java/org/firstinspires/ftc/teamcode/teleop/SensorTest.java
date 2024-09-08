package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.EmptyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.filters.ExponentialMovingAverage;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@Config
@TeleOp
public class SensorTest extends OpModePipeline {
    private DistanceSensor distSensor;

    private ExponentialMovingAverage ema = new ExponentialMovingAverage(10, 0.9);
    public static double decaying_factor = 0.9;

    public void init() {
        runMode = RobotRunMode.TELEOP;
        subsystems = new EmptyConfig();
        super.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        distSensor = hardwareMap.get(DistanceSensor.class, "distance_front");
    }

    @Override
    public void loop() {
        Robot.update();

        ema.run(distSensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("sensor", distSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("filter", ema.getEstimate());
        telemetry.addData("decaying_factor", decaying_factor);

        ema.setDecayingFactor(decaying_factor);

        telemetry.update();
    }
}