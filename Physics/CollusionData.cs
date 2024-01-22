using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PhysicsEngine.Physics;

public class CollusionData
{
    public float depth;
    public PolygonalRigidBody bodyA;
    public PolygonalRigidBody bodyB;
    public Vector2 normal;
    public Vector2 contactPoint;

    public CollusionData Clone()
    {
        return new CollusionData()
        {
            bodyA = bodyA,
            bodyB = bodyB,
            normal = normal,
            depth = depth,
            contactPoint = contactPoint
        };
    }

    public void SetContactPoint(List<Vector2> clippingResult)
    {
        Vector2 p;
        if (clippingResult.Count == 1)
        {
            p = clippingResult[0];
        }
        else if (clippingResult.Count == 2)
        {
            p = (clippingResult[0] + clippingResult[1]) / 2;
        }
        else
        {
            throw new Exception(clippingResult.Count + " contact points, invalid amount");
        }
        contactPoint = p;
    }

    public float CalculateSeparatingVelocity()
    {
        Vector2 n = -normal;
        Vector2 p = contactPoint;

        Vector2 r_ap = p - bodyA.Position;
        Vector2 r_ap_perp = new Vector2(-r_ap.Y, r_ap.X);
        Vector2 v_a = bodyA.Velocity;
        float w_a = bodyA.AngularVelocity;
        Vector2 v_ap = v_a + w_a * r_ap_perp;

        Vector2 r_bp = p - bodyB.Position;
        Vector2 r_bp_perp = new Vector2(-r_bp.Y, r_bp.X);
        Vector2 v_b = bodyB.Velocity;
        float w_b = bodyB.AngularVelocity;
        Vector2 v_bp = v_b + w_b * r_bp_perp;

        Vector2 v_ab = v_ap - v_bp;
        return Vector2.Dot(v_ab, n);
    }

    public void CalculateDepth()
    {
        Projection i1 = bodyA.ProjectOntoDirection(normal);
        Projection i2 = bodyB.ProjectOntoDirection(normal);
        depth = i1.end - i2.begin;
    }

    public void ResolvePenetration()
    {
        if (bodyA.inmovable && bodyB.inmovable) return;
        Vector2 penVector = normal * depth;
        if (bodyA.inmovable)
        {
            bodyB.Position += penVector;
            return;
        }
        if (bodyB.inmovable)
        {
            bodyA.Position -= penVector;
            return;
        }

        float mA = bodyA.mass;
        float mB = bodyB.mass;

        Vector2 ATranslation = -penVector * (mB / (mA + mB));
        Vector2 BTranslation = penVector * (mA / (mA + mB));
        bodyA.Position += ATranslation;
        bodyB.Position += BTranslation;
    }

    public void ResolveVelocityAndRotation()
    {
        if (bodyA.inmovable && bodyB.inmovable) return;

        float e = MathHelper.Min(bodyA.RESTITUTION, bodyB.RESTITUTION);
        Vector2 n = -normal;
        Vector2 p = contactPoint;

        float MA = bodyA.mass;
        float IA = bodyA.momentOfInertia;
        Vector2 v_a = bodyA.Velocity;
        float w_a = bodyA.AngularVelocity;

        float MB = bodyB.mass;
        float IB = bodyB.momentOfInertia;
        Vector2 v_b = bodyB.Velocity;
        float w_b = bodyB.AngularVelocity;

        Vector2 r_ap = p - bodyA.Position;
        Vector2 r_ap_perp = new Vector2(-r_ap.Y, r_ap.X);
        Vector2 v_ap = v_a + w_a * r_ap_perp;

        Vector2 r_bp = p - bodyB.Position;
        Vector2 r_bp_perp = new Vector2(-r_bp.Y, r_bp.X);
        Vector2 v_bp = v_b + w_b * r_bp_perp;

        Vector2 v_ab = v_ap - v_bp;

        float numerator = Vector2.Dot(-(1 + e) * v_ab, n);

        float invMA = bodyA.inmovable ? 0 : 1 / MA;
        float invMB = bodyB.inmovable ? 0 : 1 / MB;
        float invIA = bodyA.inmovable ? 0 : 1 / IA;
        float invIB = bodyB.inmovable ? 0 : 1 / IB;

        float d1 = Vector2.Dot(n, (invMA + invMB) * n);
        float d2 = Vector2.Dot(r_ap_perp, n) * Vector2.Dot(r_ap_perp, n) * invIA;
        float d3 = Vector2.Dot(r_bp_perp, n) * Vector2.Dot(r_bp_perp, n) * invIB;
        float denominator = d1 + d2 + d3;

        float j = numerator / denominator; // the impulse due to newton's laws

        bodyA.Velocity += j * invMA * n;
        bodyA.AngularVelocity += Vector2.Dot(r_ap_perp, j * n) * invIA;


        bodyB.Velocity -= j * invMB * n;
        bodyB.AngularVelocity -= Vector2.Dot(r_bp_perp, j * n) * invIB;

        // RESOLVE FRICTION

        // recalculate velocity related stuff
        v_a = bodyA.Velocity;
        w_a = bodyA.AngularVelocity;

        v_b = bodyB.Velocity;
        w_b = bodyB.AngularVelocity;

        v_ap = v_a + w_a * r_ap_perp;
        v_bp = v_b + w_b * r_bp_perp;
        v_ab = v_ap - v_bp;


        Vector2 v_plane = v_ab - Vector2.Dot(v_ab, n) * n;
        if (v_plane.Length() == 0)
        {
            return;
        }
        Vector2 t = -v_plane;
        t.Normalize();

        numerator = Vector2.Dot(-v_ab, t);
        d1 = Vector2.Dot(t, (invMA + invMB) * t);
        d2 = Vector2.Dot(r_ap_perp, t) * Vector2.Dot(r_ap_perp, t) * invIA;
        d3 = Vector2.Dot(r_bp_perp, t) * Vector2.Dot(r_bp_perp, t) * invIB;
        denominator = d1 + d2 + d3;

        float k = numerator / denominator;
        bodyA.Velocity += k * invMA * t;
        bodyA.AngularVelocity += Vector2.Dot(r_ap_perp, k * t) * invIA;


        bodyB.Velocity -= k * invMB * t;
        bodyB.AngularVelocity -= Vector2.Dot(r_bp_perp, k * t) * invIB;

    }
}


