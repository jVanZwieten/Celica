namespace Celica
{
    /// <summary>
    /// All units used within Celica are standard SI base units:
    /// length: meter (m)
    /// time: second (s)
    /// mass: kilogram (kg)
    /// 
    /// and their derived units. Angles are in radians. It is up to the user to convert all inputs to these standard units, and convert outputs back to whichever units desired. The PhysicalUnits class provides conversion factors to make converting trivial and readable.
    /// </summary>
    public static class PhysicalUnits
    {
        public const double km = 1e3;

        public const double min = 60d;
        public const double hour = 60d * min;
        public const double day = 24d * hour;

        public const double deg = Math.PI / 180d;
    }

    public static class PhysicalUnitsF
    {
        public const float km = 1e3f;

        public const float min = 60f;
        public const float hour = 60f * min;
        public const float day = 24f * hour;

        public const float deg = MathF.PI / 180f;
    }
}
