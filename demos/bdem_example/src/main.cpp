/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2023-07-18 10:51:01
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-07-18 16:29:02
 * @FilePath: \Kiri2D\demos\bdem_example\src\main.cpp
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG, All Rights Reserved.
 */
#include <kiri2d.h>

using namespace KIRI2D;

inline double clampRad(double a)
{
    return a < -KIRI_PI<double>() ? clampRad(a + 2.0 * KIRI_PI<double>()) : (a > KIRI_PI<double>() ? clampRad(a - 2.0 * KIRI_PI<double>()) : a);
}

struct Vec
{
    double x, y;

    Vec(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    Vec operator+(const Vec &a) const
    {
        return Vec(x + a.x, y + a.y);
    }

    Vec operator-(const Vec &a) const
    {
        return Vec(x - a.x, y - a.y);
    }

    Vec operator*(double a) const
    {
        return Vec(x * a, y * a);
    }

    Vec operator/(double a) const
    {
        return Vec(x / a, y / a);
    }

    double len()
    {
        return std::sqrt(x * x + y * y);
    }

    double dot(const Vec &a)
    {
        return x * a.x + y * a.y;
    }
};

struct Bond
{
    int s, j;
    double l;
    Vec d;
    double tn, tt;

    Bond(int j, int s, double l, Vec d, double tn, double tt)
        : j(j), s(s), l(l), d(d), tn(tn), tt(tt) {}
};

struct Particle
{
    double im, iI, r, q, w, wMid, T;
    Vec x, v, vMid, F;
    int c;
    std::vector<Bond> bonds;

    Particle(double im, double iI, double r, Vec x, int c)
        : im(im), iI(iI), r(r), x(x), c(c), wMid(0), w(0), q(0) {}
};

int main(int argc, char *argv[])
{

    // log system
    KiriLog::init();
    // scene renderer config
    float window_height = 1500.f;
    float window_width = 1500.f;

    auto offset = Vector2F(250.f);

    auto scene = std::make_shared<KiriScene2D<float>>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D<float>>(scene);
    auto scale_size = 1000.0;

    int numOfFrames = 200;
    double fps = 60, r = 0.02, kn = 1e7;
    std::vector<Particle> pts;

    for (double x = 0.1; x < 0.9; x += 2 * r)
    {
        for (double y = 0.4; y < 0.45; y += 2 * r)
        {
            pts.push_back(Particle(1.3e-4 / r / r, 2.6e-4 / r / r / r / r, r, Vec(x, y), 1));
        }
    }

    for (double x = 0.7; x < 0.9; x += 2 * r)
    {
        for (double y = 0.1; y < 0.3; y += 2 * r)
        {
            pts.push_back(Particle(1.3e-4 / r / r, 2.6e-4 / r / r / r / r, r, Vec(x, y), 2));
        }
    }

    for (int i = 0; i < pts.size(); i++)
    {
        for (int j = 0; j < pts.size(); j++)
        {
            if (i != j)
            {
                Vec l = pts[j].x - pts[i].x;
                double overlap = pts[i].r + pts[j].r - l.len();
                if (overlap >= -0.1 * r)
                {
                    pts[i].bonds.push_back(Bond(j, 0, 2.0 * r, l / l.len(), 0.07 * kn, 0.07 * kn));
                }
            }
        }
    }

    // boundary particles
    for (double x = r; x < 1.0; x += 2 * r)
    {
        pts.push_back(Particle(0.0, 0.0, r, Vec(x, 0.0), 0));
        pts.push_back(Particle(0.0, 0.0, r, Vec(x, 1.0), 0));
        pts.push_back(Particle(0.0, 0.0, r, Vec(0.0, x), 0));
        pts.push_back(Particle(0.0, 0.0, r, Vec(1.0, x), 0));
    }

    // sand
    for (double x = 0.2; x < 0.8; x += 2 * r)
    {
        for (double y = 0.5; y < 0.7; y += 2 * r)
        {
            pts.push_back(Particle(1.3e-4 / r / r, 2.6e-4 / r / r / r / r, r, Vec(x, y), 3));
        }
    }

    int s = static_cast<int>(1.0 / fps / std::sqrt(7.5e3 * r * r / kn)) * 10;
    double dt = 1.0 / fps / static_cast<double>(s);

    for (int frameIdx = 0; frameIdx < numOfFrames; frameIdx++)
    {

        for (int iter = 0; iter < s; iter++)
        {

            for (int i = 0; i < pts.size(); i++)
            {
                pts[i].F = Vec();
                pts[i].T = 0.0;
                pts[i].x = pts[i].x + pts[i].vMid * dt;
                pts[i].q = clampRad(pts[i].q + pts[i].wMid * dt);
            }

            for (int i = 0; i < pts.size(); i++)
            {
                // non-boundary particles
                if (pts[i].im > 1e-6)
                {
                    for (int j = 0; j < pts.size(); j++)
                    {
                        if (i != j)
                        {
                            Vec lij = pts[i].x - pts[j].x;
                            double o = pts[i].r + pts[j].r - lij.len();

                            // non-overlap
                            if (o <= 1e-12)
                            {
                                continue;
                            }
                            Vec n = lij / lij.len();
                            double a = 1.4 * std::sqrt(kn / (pts[i].im + pts[j].im));
                            pts[i].F = pts[i].F + n * (kn * o + a * (pts[j].v - pts[i].v).dot(n));
                        }
                    }

                    for (int bidx = 0; bidx < pts[i].bonds.size(); bidx++)
                    {
                        Bond &b = pts[i].bonds[bidx];
                        if (b.s != 0)
                        {
                            continue;
                        }
                        Vec l = pts[b.j].x - pts[i].x;
                        Vec n = l / l.len();
                        Vec t(-n.y, n.x);
                        double dl = l.len() - b.l;
                        double qb = std::atan2(b.d.y, b.d.x) - std::atan2(n.y, n.x);
                        double ti = clampRad(qb + pts[i].q);
                        double tj = clampRad(qb + pts[b.j].q);

                        // equation 4
                        Vec Fn = n * kn * dl;
                        Vec Ft = t * (-kn / 3.0 * r * r / l.len() * (ti + tj));
                        double T = kn / 6.0 * r * r * (tj - 3.0 * ti);
                        if ((dl > 0.0 && (Fn.len() / (2.0 * r) + std::abs(kn / 2.0 * (tj - ti))) > b.tn) || (Ft.len() / (2.0 * r) > b.tt))
                        {
                            b.s = 1;
                            continue;
                        }
                        pts[i].F = pts[i].F + Fn + Ft;
                        pts[i].T = pts[i].T + T;
                    }
                }
            }

            for (int i = 0; i < pts.size(); i++)
            {
                Vec acc = pts[i].F * pts[i].im + (pts[i].im > 1e-6 ? Vec(0, -9.8) : Vec());
                pts[i].v = pts[i].vMid + acc * 0.5 * dt;
                pts[i].vMid = pts[i].vMid + acc * dt;
                pts[i].w = pts[i].wMid + pts[i].T * pts[i].iI * 0.5 * dt;
                pts[i].wMid = pts[i].wMid + pts[i].T * pts[i].iI * dt;
            }
        }

        std::vector<KiriCircle2<float>> circles;
        for (int i = 0; i < pts.size(); i++)
        {
            Vector3F color;

            if (pts[i].c == 0)
                color = Vector3F(0.f);
            else if (pts[i].c == 1)
                color = Vector3F(1.f, 0.f, 0.f);
            else if (pts[i].c == 2)
                color = Vector3F(0.f, 1.f, 0.f);
            else if (pts[i].c == 3)
                color = Vector3F(0.f, 0.f, 1.f);

            auto mic_paint = KiriCircle2<float>(Vector2F(pts[i].x.x * scale_size, pts[i].x.y * scale_size) + offset, color, pts[i].r * scale_size);
            circles.emplace_back(mic_paint);

            // KIRI_LOG_DEBUG("pos={0},{1}; r={2}",pts[i].x.x,pts[i].x.y, pts[i].r);

            // std::fprintf(fp, "\\draw[gray, fill=%s] (%f, ", cols[pts[i].c], pts[i].x.x);
            // std::fprintf(fp, "%f) circle[radius=%f];", pts[i].x.y, pts[i].r);
        }

        scene->AddCircles(circles);
        renderer->DrawCanvas();

        renderer->SaveImages2File();

        renderer->ClearCanvas();
        scene->Clear();
    }

    return 0;
}