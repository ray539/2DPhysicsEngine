using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Diagnostics;
using static PhysicsEngine.Physics.PolygonalRigidBody;

namespace PhysicsEngine.Physics;

public struct Force
{
    public Vector2 forceVector;
    public Vector2 contactPoint;
}


public class PolygonalRigidBody
{
    public bool colliding = false;

    private List<Vector2> localPoints;
    private Vector2 position;
    private float rotation;
    
    private Vector2 velocity;
    private float angularVelocity;

    private Vector2 acceleration;
    private float angularAcceleration;

    public float DENSITY = 0.001f;
    public float RESTITUTION = 0.2f;
    public float mass;
    public float momentOfInertia;
    public bool inmovable;
    


    public Vector2 Velocity { get => velocity; set => velocity = value; }
    public float AngularVelocity { get => angularVelocity; set => angularVelocity = value; }
    public Vector2 Position { get => position; set => position = value; }
    public float Rotation { get => rotation; set => rotation = value; }
    public Vector2 Acceleration { get => acceleration;}
    public float AngularAcceleration { get => angularAcceleration; }

    public PolygonalRigidBody(List<Vector2> points, bool inmovable)
    {
        this.Position = Common.GetPolygonCentroid(points);
        this.Rotation = 0;
        this.Velocity = Vector2.Zero;
        this.AngularVelocity = 0;
        this.acceleration = Vector2.Zero;
        this.angularAcceleration = 0;
        this.mass = Common.GetPolygonArea(points) * DENSITY;
        this.momentOfInertia = Common.GetPolygonMomentOfInertia(points, DENSITY);

        this.localPoints = Common.TransformPoints(points, this.ToLocal);
        this.inmovable = inmovable;
    }

    public BoundingRect GetBoundingRect()
    {
        List<Vector2> points = GetGlobalPoints();
        float minX = float.MaxValue;
        float maxX = float.MinValue;
        float minY = float.MaxValue;
        float maxY = float.MinValue;
        foreach (Vector2 point in points)
        {
            minX = MathHelper.Min(minX, point.X);
            minY = MathHelper.Min(minY, point.Y);

            maxX = MathHelper.Max(maxX, point.X);
            maxY = MathHelper.Max(maxY, point.Y);
        }

        return new BoundingRect() { minX = minX, maxX = maxX, minY = minY, maxY = maxY};
    }


    public void EmptyForces()
    {
        this.acceleration = Vector2.Zero;
        this.angularAcceleration = 0;
    }

    /// <summary>
    /// <paramref name="force"/> force vector
    /// <paramref name="contactPoint"/> contact point in world coordinates
    /// </summary>
    public void ApplyForce(Force force)
    {
        if (this.inmovable)
        {
            return;
        }
        Vector2 deltaA = force.forceVector / this.mass;
        Vector2 r = this.position - force.contactPoint;
        Vector2 fPerp = new Vector2(-force.forceVector.Y, force.forceVector.X);
        float deltaAlpha = Vector2.Dot(r, fPerp);

        this.acceleration += deltaA;
        this.angularAcceleration += deltaAlpha;
    }

    public void Step(float dt)
    {
        this.Velocity += dt * this.Acceleration;
        this.Position += dt * this.Velocity + (1 / 2) * (dt * dt) * this.Acceleration;

        this.AngularVelocity += dt * this.AngularAcceleration;
        this.Rotation += dt * this.AngularVelocity + (1 / 2) * (dt * dt) * this.AngularAcceleration;
    }

    public List<Vector2> GetNormals()
    {
        List<Vector2> points = GetGlobalPoints();
        int n = points.Count;
        points.Add(points[0]);
        List<Vector2> normals = new List<Vector2>();
        for (int i = 0; i < n; i++) {
            Vector2 pi = points[i];
            Vector2 pi1 = points[i + 1];
            Vector2 edge = pi1 - pi;
            Vector2 norm = new (edge.Y, -edge.X);
            normals.Add(norm);
        }
        return normals;
    }

    public List<Edge> GetEdges()
    {
        List<Vector2> points = GetGlobalPoints();
        int n = points.Count;
        points.Add(points[0]);
        List<Edge> edges = new List<Edge>();
        for (int i = 0; i < n; i++)
        {
            Vector2 pi = points[i];
            Vector2 pi1 = points[i + 1];
            edges.Add(new Edge { a = pi, b = pi1 });
        }
        return edges;
    }


    public Projection ProjectOntoDirection(Vector2 direction)
    {
        direction.Normalize();
        Projection projection = new() { 
            begin = float.MaxValue, 
            end = float.MinValue,
        };
        List<Vector2> globalPoints = GetGlobalPoints();

        foreach(Vector2 point in globalPoints)
        {
            float projDist = Vector2.Dot(direction, point);

            if (projDist < projection.begin)
            {
                projection.begin = projDist;
            }
            
            if (projDist > projection.end)
            {
                projection.end = projDist;
            }
        }

        return projection;
    }

    public Vector2 ToLocal(Vector2 globalPoint)
    {
        Vector2 res = globalPoint - Position;
        Matrix rotationMatrix = Matrix.CreateRotationZ(-Rotation);
        Vector2.Transform(res, rotationMatrix);
        return res;
    }

    public Vector2 ToWorld(Vector2 localPoint)
    {
        Matrix rotationMatrix = Matrix.CreateRotationZ(Rotation);
        Vector2 res = Vector2.Transform(localPoint, rotationMatrix);
        res += Position;
        return res;
    }

    public List<Vector2> GetGlobalPoints()
    {
        return Common.TransformPoints(localPoints, this.ToWorld);
    }

}
