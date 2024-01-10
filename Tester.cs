
using Microsoft.Xna.Framework;
using PhysicsEngine.Physics;
using System.Collections.Generic;
using System.Diagnostics;

namespace PhysicsEngine;

internal class Tester
{

    public static void Main()
    {
        PolygonalRigidBody bodyA = World.GetPolygonalRigidBody(new List<Vector2>() {
                    new Vector2(0, 0),
                    new Vector2(100, 0),
                    new Vector2(100, 100),
                    new Vector2(0, 100)
                }, false);
        Debug.WriteLine(Common.GetPolygonMomentOfInertia(bodyA.GetGlobalPoints(), bodyA.DENSITY));
    }
}

