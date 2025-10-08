using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    public static class Utilities
    {
        public static double NormalizeAngleZeroToTwoPi(double rads)
        {
            rads = rads % (2 * MathF.PI);
            if (rads < 0)
                rads += 2 * MathF.PI;
            return rads;
        }

        public static float NormalizeAngleZeroToTwoPiF(float rads)
        {
            rads = rads % (2 * MathF.PI);
            if (rads < 0)
                rads += 2 * MathF.PI;
            return rads;
        }

        public static Vector Cross(this Vector a, Vector b)
        {
            if (a.Count != 3 || b.Count != 3)
                throw new ArgumentException("Cross product is only defined for 3D vectors.");
            return new DenseVector(
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            ]);
        }
    }
}
