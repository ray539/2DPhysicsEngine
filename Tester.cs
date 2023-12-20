
using Microsoft.Xna.Framework;
using PhysicsEngine.Physics;
using System.Collections.Generic;
using System.Diagnostics;

namespace PhysicsEngine;

internal class Tester
{

    public static void Main()
    {
        PolygonalRigidBody box2 = World.GetBox(0, 0, 100, 100);
        PolygonalRigidBody box1 = World.GetBox(75, 75, 100, 100);
  

        Debug.WriteLine(World.SATIntersect(box1, box2));
    }
}

