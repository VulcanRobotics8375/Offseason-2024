package org.firstinspires.ftc.teamcode.robotcorelib.math.filters;

public class KalmanFilter {

    private double x = 0; // your initial state
    private double Q = 0.1; // your model covariance
    private double R = 0.4; // your sensor covariance
    private double p = 1; // your initial covariance guess
    private double K = 1; // your initial Kalman gain guess

    private double x_previous = x;
    private double p_previous = p;
    private double u = 0;
    private double z = 0;

    private double lastMeasurement = x;

    public KalmanFilter(double Q, double R) {
        this.Q = Q;
        this.R = R;
    }

    public void run(double measurement) {
        double u = measurement - lastMeasurement;
        x = x_previous + u;

        p = p_previous + Q;

        K = p/(p + R);

        x = x + K * (measurement - x);

        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;
        lastMeasurement = measurement;
    }

    public double getEstimate() {
        return x;
    }

    public void setQ(double Q) {
        this.Q = Q;
    }

    public void setR(double R) {
        this.R = R;
    }


}