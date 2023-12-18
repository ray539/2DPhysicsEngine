using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Diagnostics;

namespace PhysicsEngine.Physics;



public class PolygonalRigidBody
{
    private List<Vector2> localPoints;
    private Vector2 position;
    private float rotation;
    
    private Vector2 velocity;
    private float angularVelocity;

    private Vector2 acceleration;
    private float angularAcceleration;

    private float mass;

    public Vector2 Velocity { get => velocity; set => velocity = value; }
    public float AngularVelocity { get => angularVelocity; set => angularVelocity = value; }
    public Vector2 Position { get => position; set => position = value; }

    public PolygonalRigidBody(List<Vector2> points)
    {
        this.Position = Common.GetPolygonCentroid(points);
        this.rotation = 0;
        this.Velocity = Vector2.Zero;
        this.AngularVelocity = 0;
        this.acceleration = Vector2.Zero;
        this.angularAcceleration = 0;
        this.mass = 1;

        this.localPoints = Common.TransformPoints(points, this.ToLocal);
    }

    public void ApplyImpulse(Vector2 impulse)
    {
        this.Velocity += impulse / this.mass;
    }


    public struct BoundingRect
    {
        public float minX;
        public float maxX;
        public float minY;
        public float maxY;
    }
    public BoundingRect GetBoundingRect()
    {
        List<Vector2> points = GetGlobalPoints();
        float minX = 100000;
        float maxX = -100000;
        float minY = 100000;
        float maxY = -100000;
        foreach (Vector2 point in points)
        {
            minX = MathHelper.Min(minX, point.X);
            minY = MathHelper.Min(minY, point.Y);
            maxX = MathHelper.Max(maxX, point.X);
            maxY = MathHelper.Max(maxY, point.Y);
        }

        return new BoundingRect() { minX = minX, maxX = maxX, minY = minY, maxY = maxY};
    }

    public void ApplyForce(Vector2 force)
    {
        this.acceleration = force / this.mass;
    }

    public void Step(float dt)
    {
        this.Velocity += dt * this.acceleration;
        this.Position += dt * this.Velocity + (1 / 2) * (dt * dt) * this.acceleration;

        this.angularVelocity += dt * this.angularAcceleration;
        this.rotation += dt * this.angularVelocity + (1 / 2) * (dt * dt) * this.angularAcceleration;
    }

    public Vector2 ToLocal(Vector2 globalPoint)
    {
        Vector2 res = globalPoint - Position;
        Matrix rotationMatrix = Matrix.CreateRotationZ(-rotation);
        Vector2.Transform(res, rotationMatrix);
        return res;
    }

    public Vector2 ToWorld(Vector2 localPoint)
    {
        Matrix rotationMatrix = Matrix.CreateRotationZ(rotation);
        Vector2 res = Vector2.Transform(localPoint, rotationMatrix);
        res += Position;
        return res;
    }

    public List<Vector2> GetGlobalPoints()
    {
        return Common.TransformPoints(localPoints, this.ToWorld);
    }

}
