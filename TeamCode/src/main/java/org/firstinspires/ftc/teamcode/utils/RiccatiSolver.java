package org.firstinspires.ftc.teamcode.utils;

/**
 * Solves the Discrete Algebraic Riccati Equation (DARE) for LQR control.
 * Supports N-dimensional state spaces using Gaussian Elimination.
 */
public class RiccatiSolver {

    private static final int MAX_ITERATIONS = 500;
    private static final double EPSILON = 1e-10;

    /**
     * Calculates the optimal LQR gain matrix K.
     * u = -K * x
     * * @param A State transition matrix (n x n)
     * @param B Control input matrix (n x m)
     * @param Q State cost matrix (n x n)
     * @param R Control cost matrix (m x m)
     * @return The optimal gain matrix K (m x n)
     */
    public static double[][] calculateLQR(double[][] A, double[][] B, double[][] Q, double[][] R) {
        int n = A.length;

        // Transpose matrices once to save cycles
        double[][] AT = MarixOperations.transpose(A);
        double[][] BT = MarixOperations.transpose(B);

        // Initialize P as Q (Dynamic Programming start)
        double[][] P = Q;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            // 1. Calculate the 'inverted term': (R + B^T * P * B)^-1
            double[][] BTP = MarixOperations.multiply(BT, P);
            double[][] BTPB = MarixOperations.multiply(BTP, B);
            double[][] invTerm = MarixOperations.invert(MarixOperations.add(R, BTPB));

            // 2. Calculate the 'gain' part: (A^T * P * B) * invTerm * (B^T * P * A)
            double[][] ATPB = MarixOperations.multiply(AT, MarixOperations.multiply(P, B));
            double[][] BTPA = MarixOperations.multiply(BT, MarixOperations.multiply(P, A));

            double[][] intermediate = MarixOperations.multiply(ATPB, invTerm);
            double[][] subtractionTerm = MarixOperations.multiply(intermediate, BTPA);

            // 3. Update P: P_next = Q + A^T * P * A - subtractionTerm
            double[][] ATPA = MarixOperations.multiply(AT, MarixOperations.multiply(P, A));

            // Note: subtraction is done by adding the negative
            double[][] nextP = MarixOperations.add(Q,
                    MarixOperations.add(ATPA, MarixOperations.multiply(subtractionTerm, -1.0))
            );

            // Check if the matrix P has stabilized (converged)
            if (isConverged(P, nextP)) {
                P = nextP;
                break;
            }
            P = nextP;
        }

        // Final Calculation for K: K = (R + B^T * P * B)^-1 * (B^T * P * A)
        double[][] finalInvTerm = MarixOperations.invert(
                MarixOperations.add(R, MarixOperations.multiply(BT, MarixOperations.multiply(P, B)))
        );
        double[][] finalBTPA = MarixOperations.multiply(BT, MarixOperations.multiply(P, A));

        return MarixOperations.multiply(finalInvTerm, finalBTPA);
    }

    private static boolean isConverged(double[][] P, double[][] nextP) {
        for (int i = 0; i < P.length; i++) {
            for (int j = 0; j < P[0].length; j++) {
                if (Math.abs(P[i][j] - nextP[i][j]) > EPSILON) return false;
            }
        }
        return true;
    }
}