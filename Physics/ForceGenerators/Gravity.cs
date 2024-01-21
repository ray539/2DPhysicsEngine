using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using PhysicsEngine;
using PhysicsEngine.Physics;
public class Gravity : ForceGenerator
{
    List<PolygonalRigidBody> bodies;

    public Gravity(List<PolygonalRigidBody> bodies)
    {
        this.bodies = bodies;
    }
    public void ApplyForces()
    {
        foreach (var body in bodies)
        {
            Vector2 g = body.mass * Common.GRAVITY * (new Vector2(0, -1));
            body.ApplyForce(new Force { forceVector = g, contactPoint = body.Position }) ;
        }
    }
}
