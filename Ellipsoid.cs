using System;


namespace rt
{
    public class Ellipsoid : Geometry
    {
        private Vector Center { get; }
        private Vector SemiAxesLength { get; }
        private double Radius { get; }
        
        
        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Color color) : base(color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            Vector d = line.Dx;
            Vector o = line.X0 - Center;

            double ax = SemiAxesLength.X * Radius;
            double ay = SemiAxesLength.Y * Radius;
            double az = SemiAxesLength.Z * Radius;

            Vector dScaled = new Vector(d.X / ax, d.Y / ay, d.Z / az);
            Vector oScaled = new Vector(o.X / ax, o.Y / ay, o.Z / az);

            double A = dScaled * dScaled;
            double B = 2.0 * (oScaled * dScaled);
            double C = oScaled * oScaled - 1.0;

            double discriminant = B * B - 4.0 * A * C;

            if (A == 0.0 || discriminant < 0.0)
            {
                return Intersection.NONE;
            }

            double sqrtD = Math.Sqrt(discriminant);
            double t1 = (-B - sqrtD) / (2.0 * A);
            double t2 = (-B + sqrtD) / (2.0 * A);

            double t = double.MaxValue;

            if (t1 >= minDist && t1 <= maxDist)
            {
                t = t1;
            }

            if (t2 >= minDist && t2 <= maxDist && t2 < t)
            {
                t = t2;
            }

            if (t == double.MaxValue)
            {
                return Intersection.NONE;
            }

            Vector pos = line.CoordinateToPosition(t);

            double nx = (pos.X - Center.X) / (ax * ax);
            double ny = (pos.Y - Center.Y) / (ay * ay);
            double nz = (pos.Z - Center.Z) / (az * az);
            Vector normal = new Vector(nx, ny, nz).Normalize();

            return new Intersection(
                valid: true,
                visible: true,
                geometry: this,
                line: line,
                t: t,
                normal: normal,
                material: Material,
                color: Color
            );
        }
    }
}
