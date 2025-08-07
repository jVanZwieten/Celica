using System.Numerics;

namespace Celica
{
    internal static class NumericalMethods
    {
        public static Vector<double> RungeKutta4(Func<Vector<double>, double, Vector<double>> dxVec, Vector<double> xVec_n, double t_n, double dt)
        {
            var K1 = dxVec(xVec_n, t_n);
            var K2 = dxVec(xVec_n + dt / 2 * K1, t_n + dt / 2);
            var K3 = dxVec(xVec_n + dt / 2 * K2, t_n + dt / 2);
            var K4 = dxVec(xVec_n + dt / 2 * K2, t_n + dt / 2);

            return xVec_n + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
        }
    }
}
