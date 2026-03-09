package org.firstinspires.ftc.teamcode.utils;

/**
 * A utility class for performing common matrix mathematics.
 */
public class MarixOperations {

    /**
     * Adds two matrices of the same dimensions.
     */
    public static double[][] add(double[][] a, double[][] b) {
        int rows = a.length;
        int cols = a[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = a[i][j] + b[i][j];
            }
        }
        return result;
    }

    /**
     * Subtracts two matrices of the same dimensions.
     */
    public static double[][] subtract(double[][] a, double[][] b) {
        int rows = a.length, cols = a[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) result[i][j] = a[i][j] - b[i][j];
        return result;
    }

    /**
     * Multiplies two matrices using the dot product of rows and columns.
     * The number of columns in 'a' must match the number of rows in 'b'.
     */
    public static double[][] multiply(double[][] a, double[][] b) {
        int rowsA = a.length;
        int colsA = a[0].length;
        int colsB = b[0].length;
        double[][] result = new double[rowsA][colsB];

        for (int i = 0; i < rowsA; i++) {
            for (int j = 0; j < colsB; j++) {
                for (int k = 0; k < colsA; k++) {
                    result[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        return result;
    }

    /**
     * Scales a matrix by a constant factor.
     */
    public static double[][] multiply(double[][] a, double scalar) {
        int rows = a.length;
        int cols = a[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = a[i][j] * scalar;
            }
        }
        return result;
    }

    /**
     * Transposes a matrix (flips it over its diagonal).
     */
    public static double[][] transpose(double[][] a) {
        int rows = a.length;
        int cols = a[0].length;
        double[][] result = new double[cols][rows];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[j][i] = a[i][j];
            }
        }
        return result;
    }

    /**
     * Calculates the determinant of a square matrix.
     */
    public static double determinant(double[][] matrix) {
        if (matrix.length == 1) return matrix[0][0];
        if (matrix.length == 2) {
            return (matrix[0][0] * matrix[1][1]) - (matrix[0][1] * matrix[1][0]);
        }

        double sum = 0;
        for (int i = 0; i < matrix[0].length; i++) {
            sum += Math.pow(-1, i) * matrix[0][i] * determinant(getSubmatrix(matrix, 0, i));
        }
        return sum;
    }

    /**
     * Helper method to get a smaller matrix by removing a specific row and column.
     */
    private static double[][] getSubmatrix(double[][] matrix, int rowToRemove, int colToRemove) {
        int n = matrix.length;
        double[][] sub = new double[n - 1][n - 1];
        int r = -1;
        for (int i = 0; i < n; i++) {
            if (i == rowToRemove) continue;
            r++;
            int c = -1;
            for (int j = 0; j < n; j++) {
                if (j == colToRemove) continue;
                sub[r][++c] = matrix[i][j];
            }
        }
        return sub;
    }

    /**
     * Calculates the dot product of two 1D vectors.
     * Formula: sum(a[i] * b[i])
     */
    public static double dotProduct(double[] a, double[] b) {
        if (a.length != b.length) {
            throw new IllegalArgumentException("Vectors must be the same length");
        }

        double result = 0;
        for (int i = 0; i < a.length; i++) {
            result += a[i] * b[i];
        }
        return result;
    }

    /**
     * Calculates the dot product of two matrices.
     * In most contexts, this is the sum of the product of all corresponding elements.
     */
    public static double dotProduct(double[][] a, double[][] b) {
        if (a.length != b.length || a[0].length != b[0].length) {
            throw new IllegalArgumentException("Matrices must have the same dimensions");
        }

        double result = 0;
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                result += a[i][j] * b[i][j];
            }
        }
        return result;
    }

    /**
     * Inverts an n x n matrix using Gaussian Elimination with Partial Pivoting.
     */
    public static double[][] invert(double[][] matrix) {
        int n = matrix.length;
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(matrix[i], 0, aug[i], 0, n);
            aug[i][i + n] = 1.0;
        }

        for (int i = 0; i < n; i++) {
            int pivot = i;
            for (int j = i + 1; j < n; j++)
                if (Math.abs(aug[j][i]) > Math.abs(aug[pivot][i])) pivot = j;

            double[] temp = aug[i]; aug[i] = aug[pivot]; aug[pivot] = temp;
            if (Math.abs(aug[i][i]) < 1e-12) throw new ArithmeticException("Singular Matrix");

            double div = aug[i][i];
            for (int j = i; j < 2 * n; j++) aug[i][j] /= div;
            for (int k = 0; k < n; k++) {
                if (k != i) {
                    double factor = aug[k][i];
                    for (int j = i; j < 2 * n; j++) aug[k][j] -= factor * aug[i][j];
                }
            }
        }
        double[][] res = new double[n][n];
        for (int i = 0; i < n; i++) System.arraycopy(aug[i], n, res[i], 0, n);
        return res;
    }

    /**
     * Linearly interpolates between matrix A and matrix B.
     * * @param a The starting matrix.
     * @param b The ending matrix.
     * @param t The interpolation factor (0.0 to 1.0).
     * @return A new matrix representing the interpolated state.
     */
    public static double[][] lerp(double[][] a, double[][] b, double t) {
        int rows = a.length;
        int cols = a[0].length;

        if (rows != b.length || cols != b[0].length) {
            throw new IllegalArgumentException("Matrices must have the same dimensions for LERP");
        }

        // Logic: Result = A + (B - A) * t
        double[][] difference = subtract(b, a);
        double[][] scaledDifference = multiply(difference, t);
        return add(a, scaledDifference);
    }
}