using System;
using System.IO;
using System.Text.RegularExpressions;

namespace rt;

public class CtScan: Geometry
{
    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public CtScan(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap) : base(Color.NONE)
    {
        _position = position;
        _scale = scale;
        _colorMap = colorMap;

        var lines = File.ReadLines(datFile);
        foreach (var line in lines)
        {
            var kv = Regex.Replace(line, "[:\\t ]+", ":").Split(":");
            if (kv[0] == "Resolution")
            {
                _resolution[0] = Convert.ToInt32(kv[1]);
                _resolution[1] = Convert.ToInt32(kv[2]);
                _resolution[2] = Convert.ToInt32(kv[3]);
            } else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(_resolution[0]*_thickness[0]*scale, _resolution[1]*_thickness[1]*scale, _resolution[2]*_thickness[2]*scale);

        var len = _resolution[0] * _resolution[1] * _resolution[2];
        _data = new byte[len];
        using FileStream f = new FileStream(rawFile, FileMode.Open, FileAccess.Read);
        if (f.Read(_data, 0, len) != len)
        {
            throw new InvalidDataException($"Failed to read the {len}-byte raw data");
        }
    }
    
    private ushort Value(int x, int y, int z)
    {
        if (x < 0 || y < 0 || z < 0 || x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }
    
    private static bool IntersectSlab(
        double origin,
        double direction,
        double min,
        double max,
        ref double tMin,
        ref double tMax)
    {
        const double eps = 1e-6;

        if (Math.Abs(direction) < eps)
        {
            if (origin < min || origin > max)
            {
                return false;
            }
            return true;
        }

        double invD = 1.0 / direction;
        double t0 = (min - origin) * invD;
        double t1 = (max - origin) * invD;

        if (t0 > t1)
        {
            double tmp = t0;
            t0 = t1;
            t1 = tmp;
        }

        if (t0 > tMin) tMin = t0;
        if (t1 < tMax) tMax = t1;

        return tMin <= tMax;
    }

    public override Intersection GetIntersection(Line line, double minDist, double maxDist)
    {
        double minX = Math.Min(_v0.X, _v1.X);
        double minY = Math.Min(_v0.Y, _v1.Y);
        double minZ = Math.Min(_v0.Z, _v1.Z);
        double maxX = Math.Max(_v0.X, _v1.X);
        double maxY = Math.Max(_v0.Y, _v1.Y);
        double maxZ = Math.Max(_v0.Z, _v1.Z);

        Vector o = line.X0;
        Vector d = line.Dx;

        double tMin = minDist;
        double tMax = maxDist;

        if (!IntersectSlab(o.X, d.X, minX, maxX, ref tMin, ref tMax) ||
            !IntersectSlab(o.Y, d.Y, minY, maxY, ref tMin, ref tMax) ||
            !IntersectSlab(o.Z, d.Z, minZ, maxZ, ref tMin, ref tMax))
        {
            return Intersection.NONE;
        }

        if (tMax < minDist || tMin > maxDist)
        {
            return Intersection.NONE;
        }

        double startT = Math.Max(tMin, minDist);
        double endT   = Math.Min(tMax, maxDist);

        double voxelStep = Math.Min(_thickness[0], Math.Min(_thickness[1], _thickness[2])) * _scale;
        if (voxelStep <= 0.0)
        {
            return Intersection.NONE;
        }

        Color color = Color.NONE;
        double alpha = 1.0;
        Vector normalAtPoint = new Vector();

        for (double t = startT; t <= endT; t += voxelStep)
        {
            Vector p = line.CoordinateToPosition(t);
            Color colorAtPoint = GetColor(p);
            if (colorAtPoint.Alpha <= 0.0) continue;
            normalAtPoint = GetNormal(p);
            color += colorAtPoint * colorAtPoint.Alpha * alpha;
            alpha *= 1.0 - colorAtPoint.Alpha;
            if (alpha <= 0.01) break;
        }
        if (color.Alpha <= 0.0)
        {
            return Intersection.NONE;
        }
        return new Intersection
        (
            valid: true,
            visible: true,
            geometry: this,
            line: line,
            t: startT,
            normal: normalAtPoint,
            material: Material.FromColor(color),
            color: color
        );
    }
    
    private int[] GetIndexes(Vector v)
    {
        return new []{
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale), 
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)};
    }
    private Color GetColor(Vector v)
    {
        int[] idx = GetIndexes(v);

        ushort value = Value(idx[0], idx[1], idx[2]);
        return _colorMap.GetColor(value);
    }

    private Vector GetNormal(Vector v)
    {
        int[] idx = GetIndexes(v);
        double x0 = Value(idx[0] - 1, idx[1], idx[2]);
        double x1 = Value(idx[0] + 1, idx[1], idx[2]);
        double y0 = Value(idx[0], idx[1] - 1, idx[2]);
        double y1 = Value(idx[0], idx[1] + 1, idx[2]);
        double z0 = Value(idx[0], idx[1], idx[2] - 1);
        double z1 = Value(idx[0], idx[1], idx[2] + 1);

        return new Vector(x1 - x0, y1 - y0, z1 - z0).Normalize();
    }
}