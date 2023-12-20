using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Diagnostics;
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

    public static PolygonalRigidBody GetBox(float x, float y, float width, float height)
    {
        List<Vector2> points = new List<Vector2>() {
            new Vector2(x, y),
            new Vector2(x + width, y),
            new Vector2(x + width, y + height),
            new Vector2(x, y + height),
        };
        PolygonalRigidBody body = new PolygonalRigidBody(points);
        return body;
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

    public static bool BoundingRectIntersect(PolygonalRigidBody bodyA, PolygonalRigidBody bodyB) {
        BoundingRect rectA = bodyA.GetBoundingRect();
        BoundingRect rectB = bodyB.GetBoundingRect();
        // sort by minX
        if (rectA.minX > rectB.minX)
        {
            (rectA, rectB) = (rectB, rectA);
        }
        if (rectA.maxX < rectB.minX) return false;

        if (rectA.minY > rectB.minY)
        {
            (rectA, rectB) = (rectB, rectA);
        }
        if (rectA.maxY < rectB.minY) return false;
        return true;
    }



    public static bool SATIntersect(PolygonalRigidBody bodyA, PolygonalRigidBody bodyB)
    {
        List<Vector2> normals = new();
        normals.AddRange(bodyA.GetNormals());
        normals.AddRange(bodyB.GetNormals());
        foreach(Vector2 n in normals)
        {
            n.Normalize();
            Interval i1 = bodyA.ProjectOntoDirection(n);
            Interval i2 = bodyB.ProjectOntoDirection(n);
            if (i2.begin < i1.begin)
            {
                (i1, i2) = (i2, i1);
            }
            if (i1.end < i2.begin)
            {
                return false;
            }
        }
        return true;
    }

    public void Step(float time)
    {
        foreach (PolygonalRigidBody rb in Bodies)
        {
            rb.Step(time);
        }


        foreach (PolygonalRigidBody rb in Bodies)
        {
            rb.colliding = false;
        }
        // check collusions using SAT
        // we have two polygons, each having their
        Debug.WriteLine(Bodies.Count);
        for (int i = 0; i < Bodies.Count; i++)
        {
            for (int j = i + 1; j < Bodies.Count; j++)
            {
                PolygonalRigidBody bodyA = Bodies[i];
                PolygonalRigidBody bodyB = Bodies[j];
                if (BoundingRectIntersect(bodyA, bodyB) && SATIntersect(bodyA, bodyB)) {
                    bodyA.colliding = true;
                    bodyB.colliding = true;
                }
            }
        }
    }

}
