using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PhysicsEngine.Physics;

public class World
{

    private List<PolygonalRigidBody> bodies;
    public List<PolygonalRigidBody> Bodies { get => bodies; private set => bodies = value; }

    public World()
    {
        this.bodies = new List<PolygonalRigidBody>();
    }

    public PolygonalRigidBody AddBox(float x, float y, float width, float height)
    {
        List<Vector2>points = new List<Vector2>() {
            new Vector2(x, y),
            new Vector2(x + width, y),
            new Vector2(x + width, y + height),
            new Vector2(x, y + height),
        };
        PolygonalRigidBody body = new PolygonalRigidBody(points);
        Bodies.Add(body);
        return body;
    }
    public PolygonalRigidBody AddPolygonalRigidBody(List<Vector2>points)
    {
        PolygonalRigidBody body = new PolygonalRigidBody(points);
        Bodies.Add(body);
        return body;
    }

    public bool BoundingRectIntersect(PolygonalRigidBody bodyA, PolygonalRigidBody bodyB) {
        PolygonalRigidBody.BoundingRect rectA = bodyA.GetBoundingRect();
        PolygonalRigidBody.BoundingRect rectB = bodyA.GetBoundingRect();
        return rectB.minX > rectA.maxX ||
               rectB.minY > rectA.maxY;
    }

    public bool SATIntersect(PolygonalRigidBody bodyA, PolygonalRigidBody bodyB)
    {

        return false;
    }

    public void Step(float time)
    {
        foreach (PolygonalRigidBody rb in Bodies)
        {
            rb.Step(time);
        }

        // check collusions using SAT
        // we have two polygons, each having their
        for (int i = 0; i < Bodies.Count; i++)
        {
            for (int j = i + 1; j < Bodies.Count; j++)
            {
                PolygonalRigidBody bodyA = Bodies[i];
                PolygonalRigidBody bodyB = Bodies[j];
                if (BoundingRectIntersect(bodyA, bodyB)) {
                    // use SAT
                }
            }
        }
    }

}
