package org.firstinspires.ftc.teamcode.TeleOp.flywheel;

public class BinarySearch {
    private double low;
    private double high;
    private double mid;

    public BinarySearch(double low, double high) {
        this.low = low;
        this.high = high;
        this.
    }

    public void goLow() {
        this.high = mid;
        this.mid = (this.low + this.high) / 2;
    }

    public void goHigh() {
        this.low = mid;
        this.mid = (this.low + this.high) / 2;
    }

    public double getLow() {
        return this.low;
    }

    public double getHigh() {
        return this.high;
    }

    public double getMid() {
        return this.mid;
    }
}