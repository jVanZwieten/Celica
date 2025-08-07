using Pq = Celica.PhysicalQuantity;

namespace Celica
{
    internal static class CelestialParameters
    {
        public static readonly Pq GravitationalParameterUnits = (Pq.m ^ 3) * (Pq.s ^ -2);
        public static readonly Pq RadiusUnits = Pq.km;

        public static readonly CelestialBody Sun =
            new(1.327124400189e20,
                1.988416e30,
                695_700);

        public static readonly CelestialBody Earth =
            new(3.9860044188e14,
                5.972168e24,
                6371.0084);

        public static readonly CelestialBody Moon =
            new(4.9048695e12,
                7.34767309e22,
                1737.4);
    }

    public struct CelestialBody
    {
        public Pq GravitationalParameter { get; }
        public Pq μ => GravitationalParameter;

        public Pq Mass { get; }
        public Pq Radius { get; }

        public CelestialBody(double gravitationalParameter, double mass, double radius)
        {
            GravitationalParameter = gravitationalParameter * CelestialParameters.GravitationalParameterUnits;
            Mass = mass * Pq.kg;
            Radius = radius * CelestialParameters.RadiusUnits;
        }
    }
}
