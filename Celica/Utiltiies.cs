using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    internal static class Utiltiies
    {
        internal static double NormalizeAngleZeroToTwoPi(double rads)
        {
            rads = rads % (2 * MathF.PI);
            if (rads < 0)
                rads += 2 * MathF.PI;
            return rads;
        }

        internal static float NormalizeAngleZeroToTwoPiF(float rads)
        {
            rads = rads % (2 * MathF.PI);
            if (rads < 0)
                rads += 2 * MathF.PI;
            return rads;
        }

        internal static Vector Cross(this Vector a, Vector b)
        {
            if (a.Count != 3 || b.Count != 3)
                throw new ArgumentException("Cross product is only defined for 3D vectors.");
            return new DenseVector(new double[]
            {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            });
        }
    }
}
