using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations.Schema;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;

namespace PhysicsEngine.Physics;

public class World
{

    private List<ForceGenerator> forceGenerators;
    private List<PolygonalRigidBody> bodies;
    public List<PolygonalRigidBody> Bodies { get => bodies; private set => bodies = value; }
    public List<ForceGenerator> ForceGenerators { get => forceGenerators; }
    public CollusionsHandler collusionsHandler;

    public List<CollusionData> Collusions { get => collusionsHandler.collusions; }

    public World()
    {
        this.bodies = new List<PolygonalRigidBody>();
        this.forceGenerators = new List<ForceGenerator>
        {
            new Gravity(this.bodies)
        };
        this.collusionsHandler = new CollusionsHandler(this);
    }

    public void AddForceGenerator(ForceGenerator forceGenerator)
    {
        this.ForceGenerators.Add(forceGenerator);
    }

    public static PolygonalRigidBody GetBox(float x, float y, float width, float height, bool inmovable)
    {
        List<Vector2> points = new List<Vector2>() {
            new Vector2(x, y),
            new Vector2(x + width, y),
            new Vector2(x + width, y + height),
            new Vector2(x, y + height),
        };
        PolygonalRigidBody body = new PolygonalRigidBody(points, inmovable);
        return body;
    }

    public PolygonalRigidBody AddBox(float x, float y, float width, float height, bool inmovable)
    {
        PolygonalRigidBody box = GetBox(x, y, width, height, inmovable);
        Bodies.Add(box);
        return box;
    }

    public static PolygonalRigidBody GetPolygonalRigidBody(List<Vector2> points, bool inmovable)
    {
        PolygonalRigidBody body = new(points, inmovable);
        return body;
    }

    public PolygonalRigidBody AddPolygonalRigidBody(List<Vector2> points, bool inmovable)
    {
        PolygonalRigidBody body = GetPolygonalRigidBody(points, inmovable);
        Bodies.Add(body);
        return body;
    }
    public void Step(float time)
    {
        foreach (var body in this.bodies)
        {
            body.EmptyForces();
        }

        foreach (var fg in ForceGenerators)
        {
            fg.ApplyForces();
        }

        foreach (PolygonalRigidBody rb in Bodies)
        {
            rb.Step(time);
        }

        this.collusionsHandler.ResolveCollusionsByMax();
    }

    public void Step(float time, int numIters)
    {
        for (int i = 0; i < numIters; i++)
        {
            Step(time / (float)numIters);
        }
    }

}
