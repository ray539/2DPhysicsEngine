
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
                });
        PolygonalRigidBody bodyB = World.GetBox(80, 80, 100, 100);
        // bodyB.Rotation = MathHelper.Pi / 4;
        World.SATIntersect(bodyB, bodyA, out CollusionData collusionData);
        collusionData = World.GetContactPoints(collusionData);
        int temp = 1;
    }
}

