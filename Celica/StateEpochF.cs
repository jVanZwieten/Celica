using System.Numerics;

namespace Celica
{
    public readonly struct StateEpochF(StateF state, float epoch)
    {
        public float Epoch { get; } = epoch;
        public StateF State { get; } = state;

        public float X => State.X;
        public float Y => State.Y;
        public float Z => State.Z;
        public float V_x => State.V_x;
        public float V_y => State.V_y;
        public float V_z => State.V_z;

        public static implicit operator StateF(StateEpochF a) => a.State;
        public static implicit operator float(StateEpochF a) => a.Epoch;
        public override string ToString() => $"t = {Epoch}: {State}";
    }

    public readonly struct StateF(Vector3 position, Vector3 velocity)
    {
        public Vector3 Position { get; } = position;
        public float X => Position.X;
        public float Y => Position.Y;
        public float Z => Position.Z;

        public Vector3 Velocity { get; } = velocity;
        public float V_x => Velocity.X;
        public float V_y => Velocity.Y;
        public float V_z => Velocity.Z;

        public StateF(float x, float y, float z, float v_x, float v_y, float v_z) : this(new Vector3(x, y, z), new Vector3(v_x, v_y, v_z)) { }

        public static StateF operator +(StateF a, StateF b) => new StateF(a.Position + b.Position, a.Velocity + b.Velocity);
        public static StateF operator *(float scalar, StateF a) => new StateF(scalar * a.Position, scalar * a.Velocity);
        public static StateF operator *(StateF a, float scalar) => scalar * a;
        public override string ToString() => $"[{X}; {Y}; {Z}; {V_x}; {V_y}; {V_z}]";
    }
}
