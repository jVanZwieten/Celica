using unit = Celica.PhysicalUnits;
using unitsF = Celica.PhysicalUnitsF;

namespace Celica
{
    public static class CelestialParameters
    {
        public static class Earth
        {
            public const double Radius = 6378.14 * unit.km;
            public const double Mu = 3.9860044188e14;
            public const double Mass = 5.97219e24;
            public const double J2 = 1082.63e-6;
            public static double R(double altitude) => Radius + altitude;

            public const double r_sunEarth = 149.6e6 * unit.km;
        }
    }
    public static class CelestialParametersF
    {
        public static class Earth
        {
            public const float Radius = 6378.14f * unitsF.km;
            public const float Mu = 3.9860044188e14f;
            public const float Mass = 5.97219e24f;
            public const float J2 = 1082.63e-6f;

            public static float R(float altitude) => Radius + altitude;
        }
    }
}
