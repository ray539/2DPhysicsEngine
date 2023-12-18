using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.ComponentModel;

namespace PhysicsEngine;

public static class Common
{
    public const uint SCREENWIDTH = 800;
    public const uint SCREENHEIGHT = 480;

    public const uint LINETHICKNESS = 5;

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

    public static List<Vector2> TransformPoints(List<Vector2> points, Func<Vector2, Vector2> transform)
    {
        List<Vector2> res = new List<Vector2>(points.Count);
        foreach (Vector2 point in points)
        {
            res.Add(transform(point));
        }

        return res;
    }
}
