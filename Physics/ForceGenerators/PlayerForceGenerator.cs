using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace PhysicsEngine.Physics.ForceGenerators;

public class PlayerForceGenerator : ForceGenerator
{
    private PolygonalRigidBody player;
    public Vector2 forceVector;

    public PlayerForceGenerator(PolygonalRigidBody player)
    {
        this.player = player;
        this.forceVector = Vector2.Zero;
    }

    public void SetForceVector(Vector2 forceVector)
    {
        this.forceVector = forceVector;
    }

    public void ApplyForces()
    {
        player.ApplyForce(new Force() { contactPoint = player.Position, forceVector = this.forceVector});
    }
}
