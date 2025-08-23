using System.Numerics;

namespace Celica
{
    public enum Unit
    {
        Meter,
        Kilogram,
        Second,
        Newton,
        Radians
    }

    public enum Prefix
    {
        Kilo = 3,
        Hecto = 2,
        Deca = 1,
        None = 0,
        Deci = -1,
        Centi = -2,
        Milli = -3,
    }

    public struct UnitArgument
    {
        public Unit Unit { get; }
        public Prefix Magnetude { get; }
        public int Order { get; }
        public UnitArgument(Unit unit, Prefix magnetude = Prefix.None, int order = 1)
        {
            Unit = unit;
            Magnetude = magnetude;
            Order = order;
        }
        public override string ToString() => $"{Unit}{(Math.Abs(Order) > 1 ? $"^{Order}" : string.Empty)}";
    }

    public struct PhysicalQuantity
    {
        public double Value { get; }
        public Dictionary<Unit, int> Units { get; }

        public PhysicalQuantity(Unit unit, double value = 1, Prefix magnetude = Prefix.None, int order = 1) : this([new(unit, magnetude, order)], value) { }
        public PhysicalQuantity(IEnumerable<UnitArgument> units, double value = 1)
        {
            double scale = Math.Pow(10, units.Sum(u => (int)u.Magnetude * u.Order));
            Value = value * scale;
            Units = units.ToDictionary(u => u.Unit, u => u.Order);
        }

        public PhysicalQuantity(double value, Dictionary<Unit, int> units)
        {
            Value = value;
            Units = units;
        }

        public static implicit operator double(PhysicalQuantity q) => q.Value;
        public override readonly string ToString() =>
            Value + string.Join(string.Empty, Units.Select(u => u.Key + (Math.Abs(u.Value) > 1 ? $"^{u.Value}" : string.Empty)));

        public static PhysicalQuantity operator *(double a, PhysicalQuantity b) => new(a * b.Value, b.Units);
        public static PhysicalQuantity operator *(PhysicalQuantity a, double b) => b * a;
        public static PhysicalQuantity operator *(PhysicalQuantity a, PhysicalQuantity b) => new(a.Value * b.Value, MultiplyUnits(a.Units, b.Units));
        public static PhysicalQuantity operator /(double a, PhysicalQuantity b) => new(a / b.Value, b.Units.ToDictionary(u => u.Key, u => -u.Value));
        public static PhysicalQuantity operator /(PhysicalQuantity a, double b) => new(a.Value / b, a.Units);
        public static PhysicalQuantity operator /(PhysicalQuantity a, PhysicalQuantity b)
        {
            Dictionary<Unit, int> newUnits = DivideUnits(a.Units, b.Units);
            return new PhysicalQuantity(a.Value / b.Value, newUnits);
        }
        public static PhysicalQuantity operator ^(PhysicalQuantity a, int exponent)
        {
            if (exponent == 0) throw new ArgumentOutOfRangeException("Exponent cannot be zero for exponentiation.");
            if (exponent < 0) return new PhysicalQuantity(1 / a.Value, a.Units.ToDictionary(u => u.Key, u => -u.Value));
            return new PhysicalQuantity(Math.Pow(a.Value, exponent), a.Units.ToDictionary(u => u.Key, u => u.Value * exponent));
        }

        public static Dictionary<Unit, int> MultiplyUnits(Dictionary<Unit, int> a, Dictionary<Unit, int> b) => a
                .Concat(b)
                .GroupBy(u => u.Key)
                .Where(g => g.Sum(u => u.Value) != 0)
                .ToDictionary(
                    g => g.Key,
                    g => g.Sum(u => u.Value)
                );

        public static Dictionary<Unit, int> DivideUnits(Dictionary<Unit, int> a, Dictionary<Unit, int> b) => MultiplyUnits(a, b.ToDictionary(u => u.Key, u => -u.Value));

        public bool OnlyHasUnit(Unit unit) => Units.Count == 1 && Units.ContainsKey(unit) && Units[unit] == 1;
        public bool HasUnits(Dictionary<Unit, int> compareUnits) =>
                        Units.Count == compareUnits.Count &&
            Units.All(u => compareUnits.TryGetValue(u.Key, out int count) && count == u.Value);

        public static PhysicalQuantity Seconds => new(Unit.Second);
        public static PhysicalQuantity s => Seconds;
        public static PhysicalQuantity Minutes => new(Unit.Second, Time.SecondsPerMinute);
        public static PhysicalQuantity min => Minutes;
        public static PhysicalQuantity Hours => new(Unit.Second, Time.SecondsPerHour);
        public static PhysicalQuantity h => Hours;
        public static PhysicalQuantity Meters => new(Unit.Meter);
        public static PhysicalQuantity m => Meters;
        public static PhysicalQuantity Kilometers => new(Unit.Meter, magnetude: Prefix.Kilo);
        public static PhysicalQuantity km => Kilometers;
        public static PhysicalQuantity Kilograms => new(Unit.Kilogram);
        public static PhysicalQuantity kg => Kilograms;


    }

    public struct Angle
    {
        public const double DegreesToRadians = Math.PI / 180.0;

        PhysicalQuantity value { get; }

        public double Degrees => (double)this / DegreesToRadians;
        public double Value => value.Value;
        public const Unit Units = Unit.Radians;

        public Angle(double radians)
        {
            value = new PhysicalQuantity(Unit.Radians, radians);
        }

        public static implicit operator double(Angle a) => a.value.Value;
        public static implicit operator Angle(double radians) => new(radians);
        public static implicit operator Angle(PhysicalQuantity q) => q.OnlyHasUnit(Unit.Radians) ? new Angle(q.Value) : throw new ArgumentException("PhysicalQuantity must be in radians to convert to Angle.");
        public static implicit operator PhysicalQuantity(Angle a) => a.value;

        public override string ToString() => value.ToString();

        public static Angle FromDegrees(double degrees) => new(degrees * DegreesToRadians);
    }

    public struct Time
    {
        public const double SecondsPerMinute = 60.0;

        public const double MinutesPerHour = 60.0;
        public const double SecondsPerHour = SecondsPerMinute * MinutesPerHour;

        public const double HoursPerDay = 24.0;
        public const double MinutesPerDay = MinutesPerHour * HoursPerDay;
        public const double SecondsPerDay = SecondsPerHour * HoursPerDay;

        public const double SecondsPerSiderealDay = 23 * SecondsPerHour + 56 * SecondsPerMinute + 4.1; // 23 hours, 56 minutes, and 4.1 seconds

        public const double DaysPerYear = 365.25; // Average length of a year in days
        public const double HoursPerYear = HoursPerDay * DaysPerYear;
        public const double MinutesPerYear = MinutesPerDay * DaysPerYear;
        public const double SecondsPerYear = SecondsPerDay * DaysPerYear;

        PhysicalQuantity value;

        public double Value => value.Value;
        public const Unit Units = Unit.Second;

        public Time(double seconds = 1)
        {
            value = new PhysicalQuantity(Unit.Second, seconds);
        }

        public static implicit operator double(Time t) => t.Value;
        public static implicit operator Time(double seconds) => new(seconds);
        public static implicit operator Time(PhysicalQuantity q) => q.OnlyHasUnit(Unit.Second) ? new Time(q.Value) : throw new ArgumentException("PhysicalQuantity must be in seconds to convert to Time.");
        public static implicit operator PhysicalQuantity(Time t) => t.value;
        public override string ToString() => value.ToString();

        public static Time Seconds => new();
        public static Time s => Seconds;
        public static Time Minutes => new(SecondsPerMinute);
        public static Time min => Minutes;
        public static Time Hours => new(SecondsPerHour);
        public static Time h => Hours;
        public static Time Days => new(SecondsPerDay);
        public static Time Years => new(SecondsPerYear);

        public double ToMinutes() => Value / SecondsPerMinute;
        public double ToHours() => Value / SecondsPerHour;
        public double ToDays() => Value / SecondsPerDay;
        public double ToYears() => Value / SecondsPerYear;
    }

    public struct Length
    {
        PhysicalQuantity value;

        public double Value => value.Value;
        public const Unit Units = Unit.Meter;

        public Length(double meters)
        {
            value = PhysicalQuantity.Meters * meters;
        }

        public static implicit operator double(Length l) => l.Value;
        public static implicit operator Length(double meters) => new(meters);
        public static implicit operator Length(PhysicalQuantity q) => q.OnlyHasUnit(Unit.Meter) ? new Length(q.Value) : throw new ArgumentException("PhysicalQuantity must be in meters to convert to Length.");
        public static implicit operator PhysicalQuantity(Length l) => l.value;
        public override string ToString() => Value.ToString();

        public static Length Meters => new();
        public static Length m => Meters;
        public static Length Kilometers => new(PhysicalQuantity.Kilometers);
        public static Length km => Kilometers;
    }

    public struct Velocity
    {
        PhysicalQuantity value;

        public double Value => value.Value;
        public static readonly Dictionary<Unit, int> Units = new() { { Unit.Meter, 1 }, { Unit.Second, -1 } };

        public Velocity(double metersPerSecond)
        {
            value = PhysicalQuantity.Meters * metersPerSecond / PhysicalQuantity.Seconds;
        }

        public static implicit operator double(Velocity v) => v.Value;
        public static implicit operator Velocity(double metersPerSecond) => new(metersPerSecond);
        public static implicit operator Velocity(PhysicalQuantity q) => q.HasUnits(Units) ? new Velocity(q.Value / PhysicalQuantity.Seconds) : throw new ArgumentException("PhysicalQuantity must be in meters per second to convert to Velocity.");
        public static implicit operator PhysicalQuantity(Velocity v) => v.value;
        public override string ToString() => Value.ToString();
        public static Velocity MetersPerSecond => new();
        public static Velocity mps => MetersPerSecond;
        public static Velocity KilometersPerSecond => new(PhysicalQuantity.Kilometers / PhysicalQuantity.Seconds);
        public static Velocity kmps => KilometersPerSecond;
    }

    public struct StateVector
    {
        Length x { get; }
        Length y { get; }
        Length z { get; }
        Velocity xDot { get; }
        Velocity yDot { get; }
        Velocity zDot { get; }

        public StateVector(Length x, Length y, Length z, Velocity xDot, Velocity yDot, Velocity zDot)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.xDot = xDot;
            this.yDot = yDot;
            this.zDot = zDot;
        }

        public static implicit operator double[](StateVector sv) => [sv.x, sv.y, sv.z, sv.xDot, sv.yDot, sv.zDot];
        public static implicit operator Vector<double>(StateVector sv) => new(sv);
    }

}
