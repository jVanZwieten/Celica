using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    public static class ClohessyWiltshire
    {
        public static Func<State, double, State> dXVecDt(double n)
        {
            Matrix A = DenseMatrix.OfArray(new double[,]
            {
                { 0, 0, 0, 1, 0, 0 },
                { 0, 0, 0, 0, 1, 0 },
                { 0, 0, 0, 0, 0, 1 },
                { 3 * n * n, 0, 0, 0, 2 * n, 0 },
                { 0, 0, 0, -2 * n, 0, 0 },
                { 0, 0, -n * n, 0, 0, 0 }
            });
            return (State state, double epoch) => new((Vector)(A * state.Vector));
        }

        public static double BoundedX(double yDot, double n) => 2 / n * yDot;
        public static double BoundedXDot(double y, double n) => n / 2 * y;
        public static double BoundedY(double xDot, double n) => 0.5 / n * xDot;
        public static double BoundedYDot(double x, double n) => -2 * n * x;
    }
}