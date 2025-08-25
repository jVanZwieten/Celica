using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Celica
{
    public readonly struct StateEpoch(State state, float epoch)
    {
        public float Epoch { get; } = epoch;
        public State State { get; } = state;

        public float X => State.X;
        public float Y => State.Y;
        public float Z => State.Z;
        public float V_x => State.V_x;
        public float V_y => State.V_y;
        public float V_z => State.V_z;

        public static implicit operator State(StateEpoch a) => a.State;
        public static implicit operator double(StateEpoch a) => a.Epoch;
        public override string ToString() => $"t = {Epoch}: {State}";
    }

    public readonly struct State(Vector3 position, Vector3 velocity)
    {
        public Vector3 Position { get; } = position;
        public float X => Position.X;
        public float Y => Position.Y;
        public float Z => Position.Z;

        public Vector3 Velocity { get; } = velocity;
        public float V_x => Velocity.X;
        public float V_y => Velocity.Y;
        public float V_z => Velocity.Z;

        public State(float x, float y, float z, float v_x, float v_y, float v_z) : this(new Vector3(x, y, z), new Vector3(v_x, v_y, v_z)) { }

        public static State operator +(State a, State b) => new State(a.Position + b.Position, a.Velocity + b.Velocity);
        public static State operator *(float scalar, State a) => new State(scalar * a.Position, scalar * a.Velocity);
        public static State operator *(State a, float scalar) => scalar * a;
        public override string ToString() => $"[{X}; {Y}; {Z}; {V_x}; {V_y}; {V_z}]";
    }
}
