package org.firstinspires.ftc.teamcode.TeleOp.flywheel;

public class Matrix {
    private int rows;
    private int cols;
    private double[][] entries;

    public Matrix(double[][] entries) {
        if (entries.length == 0 || entries[0].length == 0) {
            throw new IllegalArgumentException();
        }
        this.rows = entries.length;
        this.cols = entries[0].length;
        this.entries = entries;
    }

    private Matrix(int rows, int cols) {
        if (rows <= 0 || cols <= 0) {
            throw new IllegalArgumentException();
        }
        this.rows = rows;
        this.cols = cols;
        this.entries = new double[rows][cols];
    }

    public String toString() {
        StringBuilder result = new StringBuilder();
        for (int i = 0; i < this.rows; i++) {
            result.append("\n|");
            for (int j = 0; j < this.cols; j++) {
                if (j != 0) {
                    result.append(" ");
                }
                result.append(this.entries[i][j]);
            }
            result.append("|");
        }
        return result.substring(1);
    }

    public Matrix times(Matrix that) {
        if (this.cols != that.rows) {
            throw new ArithmeticException();
        }
        Matrix result = new Matrix(this.rows, that.cols);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < that.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    result.entries[i][j] += this.entries[i][k] * that.entries[k][j];
                }
            }
        }
        return result;
    }

    public Matrix times(double scalar) {
        Matrix result = new Matrix(this.rows, this.cols);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                result.entries[i][j] = this.entries[i][j] * scalar;
            }
        }
        return result;
    }

    public Matrix t() {
        Matrix result = new Matrix(this.cols, this.rows);
        for (int i = 0; i < this.cols; i++) {
            for (int j = 0; j < this.rows; j++) {
                result.entries[i][j] = this.entries[j][i];
            }
        }
        return result;
    }

    public Matrix inverse() {
        if (this.rows != 2 || this.cols != 2) {
            throw new IllegalStateException();
        }
        double a = this.entries[0][0];
        double b = this.entries[0][1];
        double c = this.entries[1][0];
        double d = this.entries[1][1];
        double det = a * d - b * c;
        return new Matrix(new double[][]{{d, -b}, {-c, a}}).times(1 / det);
    }

    public static void main(String[] args) {
        double[] x = {1.4, 2.1, 2.7, 3.4, 4.2};
        double[] v = {5.3, 6.1, 6.8, 7.5, 8.2};

        double[][] a = new double[x.length][2];
        for (int i = 0; i < x.length; i++) {
            a[i][0] = 1 / x[i];
            a[i][1] = 1 / (x[i] * x[i]);
        }
        Matrix A = new Matrix(a);

        double[][] b = new double[v.length][1];
        for (int i = 0; i < v.length; i++) {
            b[i][0] = 1 / (v[i] * v[i]);
        }
        Matrix B = new Matrix(b);

        System.out.println(A.t().times(A).inverse().times(A.t()).times(B));
    }
}
