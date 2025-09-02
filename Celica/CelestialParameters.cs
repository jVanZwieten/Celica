using units = Celica.PhysicalUnitsF;

namespace Celica
{
    public static class CelestialParameters
    {
        public static class Earth
        {
            public const float Radius = 6378.14f * units.km; // m
            public const float Mu = 3.9860044188e14f; // m^3/s^2
            public const float Mass = 5.97219e24f; // kg
            public const float J2 = 1082.63e-6f;

            public static float R(float altitude) => Radius + altitude;
        }
    }
}
