using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;
using PhysicsEngine;
using PhysicsEngine.Physics;
public class Collusions
{
    private World world;
    public Collusions()
    {

    }

    public static bool BoundingRectIntersect(PolygonalRigidBody bodyA, PolygonalRigidBody bodyB)
    {
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


    // get minPenetration AND the translation vector
    // enough for everything except rotation
    public static bool SATIntersect(PolygonalRigidBody bodyA, PolygonalRigidBody bodyB, out CollusionData collusionData)
    {
        List<Vector2> normals = new();
        normals.AddRange(bodyA.GetNormals());
        normals.AddRange(bodyB.GetNormals());

        // get the vertices relevant to minimum penetration
        // check if the vertices are actually in the other polygon
        collusionData = new CollusionData
        {
            depth = float.MaxValue
        };

        foreach (Vector2 n in normals)
        {
            n.Normalize();
            Projection i1 = bodyA.ProjectOntoDirection(n);
            Projection i2 = bodyB.ProjectOntoDirection(n);
            if (i2.begin < i1.begin)
            {
                (i1, i2) = (i2, i1);
                (bodyA, bodyB) = (bodyB, bodyA);
            }
            float thisDepth = i1.end - i2.begin;
            if (thisDepth < 0) return false;

            if (thisDepth < collusionData.depth)
            {
                collusionData.depth = thisDepth;
                collusionData.normal = n;

                collusionData.bodyA = bodyA;

                collusionData.bodyB = bodyB;

            }
        }
        return true;
    }

    public static Edge GetBestEdge(PolygonalRigidBody body, Vector2 normal)
    {
        // get furthest point along normal
        normal.Normalize();
        int maxIndex = -1;
        float furthestDistance = float.MinValue;
        List<Vector2> points = body.GetGlobalPoints();

        for (int i = 0; i < points.Count; i++)
        {
            Vector2 point = points[i];
            float d = Vector2.Dot(point, normal);
            if (d > furthestDistance)
            {
                furthestDistance = d;
                maxIndex = i;
            }
        }
        Vector2 vPrev = points[Common.Mod(maxIndex - 1, points.Count)];
        Vector2 v = points[Common.Mod(maxIndex, points.Count)];
        Vector2 vNext = points[Common.Mod(maxIndex + 1, points.Count)];
        Edge e1 = new Edge() { a = vPrev, b = v };
        Edge e2 = new Edge() { a = v, b = vNext };
        if (Math.Abs(Vector2.Dot(e1.GetVector(), normal)) < Math.Abs(Vector2.Dot(e2.GetVector(), normal)))
        {
            return e1;
        }
        else
        {
            return e2;
        }
    }

    public static List<Vector2> Clip(Vector2 direction, Vector2 p1, Vector2 p2, float a)
    {
        List<Vector2> result = new();
        direction.Normalize();
        float d1 = Vector2.Dot(direction, p1) - a;
        float d2 = Vector2.Dot(direction, p2) - a;
        if (d1 >= 0) result.Add(p1);
        if (d2 >= 0) result.Add(p2);
        if (d1 * d2 <= 0)
        {
            float t = Math.Abs(d1) / (Math.Abs(d1) + Math.Abs(d2));
            Vector2 intersection = p1 + t * (p2 - p1);
            result.Add(intersection);
        }
        return result;

    }

    public static CollusionData GetContactPoints(CollusionData initialData)
    {
        CollusionData finalData = initialData.Clone();

        Vector2 n = initialData.normal;
        float depth = initialData.depth;
        PolygonalRigidBody bodyA = initialData.bodyA;
        PolygonalRigidBody bodyB = initialData.bodyB;

        // get best edge of both bodies
        Edge e1 = GetBestEdge(bodyA, n);
        Edge e2 = GetBestEdge(bodyB, -n);
        // set reference and incident
        Edge refEdge;
        Edge incident;
        if (Math.Abs(Vector2.Dot(e1.GetVector(), n)) < Math.Abs(Vector2.Dot(e2.GetVector(), n)))
        {
            refEdge = e1;
            incident = e2;
        }
        else
        {
            refEdge = e2;
            incident = e1;
            n = -n;
        }
        // clip along ab
        Vector2 direction = refEdge.b - refEdge.a;

        direction.Normalize();
        float a = Vector2.Dot(direction, refEdge.a);
        List<Vector2> clippedPoints = Clip(direction, incident.a, incident.b, a);

        float b = Vector2.Dot(-direction, refEdge.b);
        clippedPoints = Clip(-direction, clippedPoints[0], clippedPoints[1], b);

        // third direction is normal pointing in
        List<Vector2> result = new();
        Vector2 refNormal = new Vector2(-direction.Y, direction.X);

        float c = Vector2.Dot(refNormal, refEdge.a);

        if (Vector2.Dot(clippedPoints[0], refNormal) - c >= 0) result.Add(clippedPoints[0]);
        if (Vector2.Dot(clippedPoints[1], refNormal) - c >= 0) result.Add(clippedPoints[1]);

        finalData.SetContactPoint(result);
        return finalData;
    }

    public List<CollusionData> collusions;


}
