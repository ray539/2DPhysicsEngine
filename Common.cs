using Microsoft.Xna.Framework;
using PhysicsEngine.Physics;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.ComponentModel.Design;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;

namespace PhysicsEngine;

public struct Projection
{
    public float begin;
    public float end;

    public override string ToString()
    {
        return String.Format("begin: {0}, end: {1}", begin, end);
    }
}

public struct BoundingRect
{
    public float minX;
    public float maxX;
    public float minY;
    public float maxY;

    public override string ToString()
    {
        return String.Format("minX: {0}, minY: {1}, maxX: {2}, maxY: {3}", minX, minY, maxX, maxY);
    }
}

public struct Edge
{
    public Vector2 a;
    public Vector2 b;
    public readonly float Length()
    {
        return (a - b).Length();
    }

    public readonly Vector2 GetVector()
    {
        return b - a;
    }

    public readonly Vector2 GetNormal()
    {
        Vector2 v = GetVector();
        return new Vector2(v.Y, -v.X);
    }
}

public class CollusionData
{
    public float depth;
    public PolygonalRigidBody bodyA;
    public PolygonalRigidBody bodyB;
    public Vector2 normal;
    public Vector2 contactPoint;

    public CollusionData Clone()
    {
        return new CollusionData() {
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
        this.contactPoint = p;
    }

    public float CalculateSeparatingVelocity()
    {
        Vector2 n = -this.normal;
        Vector2 p = this.contactPoint;

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
        Vector2 n = -this.normal;
        Vector2 p = this.contactPoint;
        
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

        float j = numerator / denominator;
        bodyA.Velocity += j * invMA * n;
        bodyB.Velocity -= j * invMB * n;
        bodyA.AngularVelocity += Vector2.Dot(r_ap_perp, j * n) * invIA;
        bodyB.AngularVelocity -= Vector2.Dot(r_bp_perp, j * n) * invIB;
    }
}


public static class Common
{
    public const uint SCREENWIDTH = 800;
    public const uint SCREENHEIGHT = 480;
    public const float GRAVITY = 900;


    public const uint LINETHICKNESS = 2;

    public static bool PAUSE = false;

    public static float GetPolygonArea(List<Vector2> polygon_)
    {
        float A = 0;
        float n = polygon_.Count;
        List<Vector2> polygon = new List<Vector2>(polygon_);
        polygon.Add(polygon_[0]);
        for (int i = 0; i < n; i++)
        {
            float xi = polygon[i].X;
            float xi1 = polygon[i + 1].X;
            float yi = polygon[i].Y;
            float yi1 = polygon[i + 1].Y;
            A += xi * yi1 - xi1 * yi;
        }
        A = A / 2;
        return A;
    }

    public static Vector2 GetPolygonCentroid(List<Vector2> polygon_)
    {
        float A = GetPolygonArea(polygon_);
        int n = polygon_.Count;
        List<Vector2> polygon = new List<Vector2>(polygon_);
        polygon.Add(polygon_[0]);
        float Sx = 0;
        for (int i = 0; i < n; i++)
        {
            float xi = polygon[i].X;
            float xi1 = polygon[i + 1].X;
            float yi = polygon[i].Y;
            float yi1 = polygon[i + 1].Y;
            Sx += (xi + xi1) * (xi * yi1 - xi1 * yi);
        }
        float xc = Sx / (6 * A);
        float Sy = 0;
        for (int i = 0; i < n; i++)
        {
            float xi = polygon[i].X;
            float xi1 = polygon[i + 1].X;
            float yi = polygon[i].Y;
            float yi1 = polygon[i + 1].Y;
            Sy += (yi + yi1) * (xi * yi1 - xi1 * yi);
        }
        float yc = Sy / (6 * A);
        return new Vector2(xc, yc);
    }

    public static float GetPolygonMomentOfInertia(List<Vector2> polygon_, float density)
    {
        int n = polygon_.Count;
        float A = GetPolygonArea(polygon_);
        Vector2 centroid = GetPolygonCentroid(polygon_);

        List<Vector2> polygon = new List<Vector2>(polygon_.Count);
        foreach(Vector2 v in polygon_)
        {
            polygon.Add(v - centroid);
        }
        polygon.Add(polygon[0]);

        float I = 0;
        for (int i = 0; i < n; i++)
        {
            float xi = polygon[i].X;
            float xi1 = polygon[i + 1].X;
            float yi = polygon[i].Y;
            float yi1 = polygon[i + 1].Y;
            I += (xi * yi1 - xi1 * yi) * (xi * xi + xi * xi1 + xi1 * xi1 + yi * yi + yi * yi1 + yi1 * yi1);
        }
        I = I / (12) * density;
        return I;
    }

    public static List<Vector2> TransformPoints(List<Vector2> points, Func<Vector2, Vector2> transform)
    {
        List<Vector2> res = new List<Vector2>(points.Count);
        foreach (Vector2 point in points)
        {
            res.Add(transform(point));
        }

        return res;
    }

    public static Vector2 Project(Vector2 v, Vector2 direction)
    {
        direction.Normalize();
        float l = Vector2.Dot(v, direction);
        return l * direction;
    }

    public static float PointToEdgeDistance(Vector2 point, Edge edge)
    {
        Vector2 edgeDirection = edge.b - edge.a;
        Vector2 vPoint = point - edge.a;
        edgeDirection.Normalize();
        float l = Vector2.Dot(vPoint, edgeDirection);
        if (l < 0)
        {
            l = 0;
        }
        float length = edge.Length();
        if (l > length)
        {
            l = length;
        }
        Vector2 projection = l * edgeDirection;
        return (vPoint - projection).Length();
    }

    public static bool FloatEquals(float a, float b)
    {
        return Math.Abs(a - b) < 0.0001;
    }

    public static int Mod(int a, int m)
    {
        if (a > 0) return a % m;
        if (a == 0) return 0;
        return a % m + m;
    }

}
