using System;

namespace rt
{
    class RayTracer(Geometry[] geometries, Light[] lights)
    {
        private double ImageToViewPlane(int n, int imgSize, double viewPlaneSize)
        {
            return -n * viewPlaneSize / imgSize + viewPlaneSize / 2;
        }

        private Intersection FindFirstIntersection(Line ray, double minDist, double maxDist)
        {
            var intersection = Intersection.NONE;

            foreach (var geometry in geometries)
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }

            return intersection;
        }

        private bool IsLit(Vector point, Light light)
        {
            Vector toLight = new Vector(light.Position - point);
            double distanceToLight = toLight.Length();
            if (distanceToLight <= 0.0)
            {
                return false;
            }
            toLight.Normalize();
            double epsilon = 1e-4;
            Vector origin = point + toLight * epsilon;
            var shadowRay = new Line(origin, light.Position);
            var intersection = FindFirstIntersection(shadowRay, 0.0, distanceToLight - epsilon);
            return !intersection.Valid || !intersection.Visible;
        }

        public void Render(Camera camera, int width, int height, string filename)
        {
            var background = new Color(0.2, 0.2, 0.2, 1.0);

            var image = new Image(width, height);
            Vector right = (camera.Up ^ camera.Direction).Normalize();

            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    double x = ImageToViewPlane(i, width,  camera.ViewPlaneWidth);
                    double y = ImageToViewPlane(j, height, camera.ViewPlaneHeight);
                    Vector pixelPosition = camera.Position + camera.Direction * camera.ViewPlaneDistance + right * x + camera.Up * y;
                    Line ray = new Line(camera.Position, pixelPosition);
                    Intersection intersection = FindFirstIntersection(ray, camera.FrontPlaneDistance, camera.BackPlaneDistance);

                    Color pixelColor = background;
                    if (intersection.Valid && intersection.Visible)
                    {
                        Color color = Color.NONE;

                        Vector V = intersection.Position;
                        Vector N = intersection.Normal;
                        Material mat = intersection.Material;

                        foreach (var light in lights)
                        {
                            color += mat.Ambient * light.Ambient;

                            if (IsLit(V, light))
                            {
                                Vector T = (light.Position - V).Normalize();
                                double diffFact = N * T;
                                if (diffFact > 0.0)
                                {
                                    color += mat.Diffuse * light.Diffuse * diffFact;
                                }

                                Vector E = (camera.Position - V).Normalize();
                                Vector R = (N * (2.0 * (N * T)) - T).Normalize();
                                double specFact = R * E;
                                if (specFact > 0.0)
                                {
                                    double spec = Math.Pow(specFact, mat.Shininess);
                                    color += mat.Specular * light.Specular * spec;
                                }
                            }
                        }
                        pixelColor = color * intersection.Color;
                    }
                    image.SetPixel(i, j, pixelColor);
                }
            }

            image.Store(filename);
        }
    }
}