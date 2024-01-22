using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;
using PhysicsEngine.Physics;
public class CollusionsHandler
{
    private World world;
    public List<CollusionData> collusions;
    public CollusionsHandler(World world)
    {
        this.world = world;
        this.collusions = new List<CollusionData>();
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

    /// <summary>
    /// returns contact points given a collusionData object.
    /// If it turns out the objects actually aren't colliding, null is returned
    /// </summary>
    /// <param name="initialData"></param>
    /// <returns></returns>
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
        if (clippedPoints.Count < 2) return null;

        float b = Vector2.Dot(-direction, refEdge.b);
        clippedPoints = Clip(-direction, clippedPoints[0], clippedPoints[1], b);

        if (clippedPoints.Count < 2) return null;
        // third direction is normal pointing in
        List<Vector2> result = new();
        Vector2 refNormal = new Vector2(-direction.Y, direction.X);
        float c = Vector2.Dot(refNormal, refEdge.a);
        if (Vector2.Dot(clippedPoints[0], refNormal) - c >= 0) result.Add(clippedPoints[0]);
        if (Vector2.Dot(clippedPoints[1], refNormal) - c >= 0) result.Add(clippedPoints[1]);

        if (result.Count == 0) return null;

        finalData.SetContactPoint(result);
        return finalData;
    }


    private void GetCollusions()
    {
        foreach (PolygonalRigidBody rb in world.Bodies)
        {
            rb.colliding = false;
        }
        this.collusions = new();
        for (int i = 0; i < world.Bodies.Count; i++)
        {
            for (int j = i + 1; j < world.Bodies.Count; j++)
            {
                PolygonalRigidBody bodyA = world.Bodies[i];
                PolygonalRigidBody bodyB = world.Bodies[j];
                if (BoundingRectIntersect(bodyA, bodyB) && SATIntersect(bodyA, bodyB, out CollusionData collusionData))
                {
                    CollusionData res = GetContactPoints(collusionData);
                    if (res != null)
                    {
                        this.collusions.Add(res);
                    }
                    
                }
            }
        }
    }

    private CollusionData GetFastestContact(out float sepVel)
    {

        // get the most negative sep. velocity
        float mostNegSepVel = float.MaxValue;
        CollusionData ret = null;
        foreach (CollusionData c in this.collusions)
        {
            float thisSepVel = c.CalculateSeparatingVelocity();
            if (thisSepVel < mostNegSepVel)
            {
                mostNegSepVel = thisSepVel;
                ret = c;
            }
        }
        sepVel = mostNegSepVel;
        return ret;
    }

    private void ResolveVelocitiesByMax()
    {
        int numIters = 0;
        int maxIters = this.collusions.Count * 2;
        while (numIters < maxIters)
        {
            float mostNegSepVel;
            CollusionData c = GetFastestContact(out mostNegSepVel);
            if (mostNegSepVel >= 0) return;
            c.ResolveVelocityAndRotation();
            numIters++;
        }
    }

    private CollusionData GetMaxDepthContact()
    {
        // return contact with max depth
        float max = float.MinValue;
        CollusionData ret = null;
        foreach (CollusionData c in this.collusions)
        {
            if (c.depth > max)
            {
                max = c.depth;
                ret = c;
            }
        }
        return ret;
    }

    private void ResolveInterpenetrationByMax()
    {
        int numIters = 0;
        int maxIters = this.collusions.Count * 2;
        while (numIters < maxIters)
        {
            // recalculate depths for all collusions
            foreach (var c_ in this.collusions)
            {
                c_.CalculateDepth();
            }
            CollusionData c = GetMaxDepthContact(); // contact with maximum depth
            if (c.depth <= 0) break;
            c.ResolvePenetration(); // moves the bodies to resolve the interpenetration
            numIters++;
        }
    }

    private void ResolveVelocitiesSimple()
    {
        for (int i = 0; i < 2; i++)
        {
            foreach (var c in this.collusions)
            {
                c.ResolveVelocityAndRotation();
            }
        }
    }

    private void ResolveInterpenetrationSimple()
    {
        for (int i = 0; i < 2; i++)
        {
            foreach (var c in this.collusions)
            {
                c.CalculateDepth();
                c.ResolvePenetration();
            }
        }

    }

    public void ResolveCollusionsSimple()
    {
        GetCollusions();
        ResolveInterpenetrationSimple();
        ResolveVelocitiesSimple();
    }

    public void ResolveCollusionsByMax()
    {
        GetCollusions();
        ResolveInterpenetrationByMax();
        ResolveVelocitiesByMax();
    }


}
