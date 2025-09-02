using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Celica
{
    internal static class Utiltiies
    {
        internal static float NormalizeAngleZeroToTwoPi(float rads)
        {
            rads = rads % (2 * MathF.PI);
            if (rads < 0)
                rads += 2 * MathF.PI;
            return rads;
        }
    }
}
