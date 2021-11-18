/***
 * @Author: Xu.WANG
 * @Date: 2021-03-19 22:04:26
 * @LastEditTime: 2021-11-10 21:30:54
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\src\kiri_pbs_cuda\emitter\cuda_volume_emitter.cpp
 */
#include <random>
#include <kiri_pbs_cuda/emitter/cuda_volume_emitter.cuh>
namespace KIRI
{

    void CudaVolumeEmitter::BuildSphVolume(SphVolumeData &data, float2 lowest, int2 vsize, float particleRadius, float3 color)
    {
        if (!bEnable)
            return;

        float offset = 2.f * particleRadius;
        for (size_t i = 0; i < static_cast<size_t>(vsize.x); ++i)
        {
            for (size_t j = 0; j < static_cast<size_t>(vsize.y); ++j)
            {
                float2 p = make_float2(lowest.x + i * offset, lowest.y + j * offset);

                data.pos.emplace_back(p);
                data.col.emplace_back(color);
            }
        }
    }

    void CudaVolumeEmitter::BuildUniDemVolume(DemVolumeData &data, float2 lowest, int2 vsize, float particleRadius, float3 color, float mass, float jitter)
    {
        if (!bEnable)
            return;

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<> dist(-1.f, 1.f);

        float offset = 2.f * particleRadius;
        for (size_t i = 0; i < static_cast<size_t>(vsize.x); ++i)
        {
            for (size_t j = 0; j < static_cast<size_t>(vsize.y); ++j)
            {
                float2 p = make_float2(lowest.x + i * offset, lowest.y + j * offset);

                data.pos.emplace_back(p + jitter * normalize(make_float2(dist(rndEngine), dist(rndEngine))));
                // data.pos.emplace_back(p);
                data.col.emplace_back(color);
                data.mass.emplace_back(mass);
            }
        }
    }

    void CudaVolumeEmitter::BuildDemUniShapeVolume(
        DemShapeVolumeData &data,
        Vec_Float3 shape,
        float3 color,
        float mass,
        float2 offset)
    {
        if (!bEnable)
            return;

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<> dist(-1.f, 1.f);

        data.minRadius = Huge<size_t>();
        for (size_t i = 0; i < shape.size(); i++)
        {
            float radius = shape[i].z;
            auto jitter = 1e-6f * normalize(make_float2(dist(rndEngine), dist(rndEngine)));
            data.pos.emplace_back(make_float2(shape[i].x, shape[i].y) + offset + jitter);
            data.col.emplace_back(color);
            data.radius.emplace_back(radius);
            data.mass.emplace_back(mass);
            data.minRadius = std::min(radius, data.minRadius);
        }
    }

    void CudaVolumeEmitter::BuildMRDemShapeVolume(
        DemShapeVolumeData &data,
        float density,
        Vec_Float3 shape,
        float3 color,
        float2 offset)
    {
        if (!bEnable)
            return;

        std::random_device seedGen;
        std::default_random_engine rndEngine(seedGen());
        std::uniform_real_distribution<> dist(-1.f, 1.f);

        data.minRadius = Huge<float>();
        data.maxRadius = Tiny<float>();
        for (size_t i = 0; i < shape.size(); i++)
        {
            auto radius = shape[i].z / 3000.f;
            auto mass = density * std::powf(radius * 2.f, 2.f);

            data.pos.emplace_back(make_float2(shape[i].x, shape[i].y) / 3000.f + offset);
            data.col.emplace_back(color);
            data.radius.emplace_back(radius);
            data.mass.emplace_back(mass);
            data.minRadius = std::min(radius, data.minRadius);
            data.maxRadius = std::max(radius, data.maxRadius);
        }
    }

    void CudaVolumeEmitter::BuildRndNSDemBoxVolume(
        DemNSBoxVolumeData &data,
        float2 lower,
        float2 upper,
        float coef_mean,
        float coef_fuzz,
        int max_number,
        const std::vector<NSPackPtr> &ns_types)
    {
        std::vector<NSPack> ns_packs;
        std::vector<float> bound_radius;

        float max_radius = 0.f;

        data.min_radius = Huge<float>();
        data.max_radius = Tiny<float>();

        // define random engine
        std::random_device engine;
        std::mt19937 gen(engine());
        std::uniform_real_distribution<float> dist(0.0, 1.0);

        for (const NSPackPtr &nstyp : ns_types)
        {
            NSPack ns(*nstyp);

            float rand = dist(gen);
            float radius = coef_mean * (2.f * (rand - 0.5f) * coef_fuzz + 1.f); // uniform dist

            ns.UpdateDefaultTypeRadius(radius);

            // convert pack model coord to world coordinate
            ns.Translate(ns.MidPoint());
            ns_packs.emplace_back(ns);

            float r = 0.f;
            for (const auto &s : ns.GetPack())
                r = fmaxf(r, length(s.center) + s.radius);

            bound_radius.emplace_back(r);

            float2 tmp_min, tmp_max;
            ns.AABB(tmp_min, tmp_max);
            for (const auto &s : ns.GetPack())
                max_radius = fmaxf(max_radius, s.radius);
        }

        // ns_packs generator
        std::list<ns_sphere_gen> ns_gens;

        const auto max_attempt = 200;
        int pack_num = 0;
        while (pack_num < max_number || max_number < 0)
        {
            int ns_idx = (int)(dist(gen) * (ns_packs.size() - 1e-20f));
            int tries = 0;
            while (true)
            {
                float2 diagonal = upper - lower;
                float2 pos = make_float2(dist(gen) * diagonal.x, dist(gen) * diagonal.y) + lower;
                float3 color = make_float3(dist(gen), dist(gen), dist(gen));

                float angle = dist(gen) * KIRI_PI;

                // copy the packing and rotate
                NSPack ns_trans(ns_packs[ns_idx]);
                ns_trans.RotateAroundOrigin(angle);
                ns_trans.Translate(pos);
                ns_trans.SetColor(color);
                const float &rad(bound_radius[ns_idx]);

                ns_sphere_gen ns_gen_i; // to be used later, but must be here because of goto's

                // check overlap: box margin
                float2 _upper = fmaxf(pos + rad * ones2(), upper);
                float2 _lower = fminf(pos - rad * ones2(), lower);
                if (_upper != upper || _lower != lower)
                {
                    for (const auto &s : ns_trans.GetPack())
                    {
                        float2 _sUpper = fmaxf(s.center + s.radius * ones2(), upper);
                        float2 _sLower = fminf(s.center - s.radius * ones2(), lower);
                        if (_sUpper != upper || _sLower != lower)
                        {
                            goto overlap;
                        }
                    }
                }
                // check overlaps: other ns_packs
                for (const ns_sphere_gen &ns_gen : ns_gens)
                {
                    bool detailedCheck = false;
                    // check overlaps between individual spheres and bounding sphere of the other clump[

                    if (lengthSquared(pos - ns_gen.center) < std::powf(rad + ns_gen.radius, 2.f))
                    {

                        for (const auto &s : ns_trans.GetPack())
                        {
                            if (std::powf(s.radius + ns_gen.radius, 2.f) > lengthSquared(s.center - ns_gen.center))
                            {
                                detailedCheck = true;
                                break;
                            }
                        }
                    }
                    // check sphere-by-sphere, since bounding spheres did overlap
                    if (detailedCheck)
                    {
                        for (const auto &s : ns_trans.GetPack())
                        {
                            for (int id = ns_gen.min_id; id <= ns_gen.max_id; id++)
                            {
                                if (lengthSquared(s.center - data.sphere_data[id].center) < std::powf(s.radius + data.sphere_data[id].radius, 2.f))
                                {
                                    goto overlap;
                                }
                            }
                        }
                    }
                }

                ns_gen_i.ns_id = pack_num;
                ns_gen_i.center = pos;
                ns_gen_i.radius = rad;
                ns_gen_i.min_id = data.sphere_data.size();
                ns_gen_i.max_id = data.sphere_data.size() + ns_trans.GetPack().size() - 1;

                // for clump
                non_spherical_particles ns_packs_data;
                float ns_mass = 0.f;
                float2 ns_moment = make_float2(0.f);
                float inertia_tensor = 0.f;

                int sub_num = ns_trans.GetPack().size();
                for (size_t i = 0; i < sub_num; i++)
                {
                    // std::cout << "realPos=" << ns_trans.GetPack()[i].center.x << "," << ns_trans.GetPack()[i].center.y << "," << ns_trans.GetPack()[i].center.z << std::endl;
                    data.sphere_data.emplace_back(ns_sphere_data(ns_trans.GetPack()[i].center, ns_trans.GetPack()[i].radius, ns_trans.GetPack()[i].color, ns_gen_i.ns_id));

                    float volume = (4.f / 3.f) * KIRI_PI * std::powf(ns_trans.GetPack()[i].radius, 3.f);

                    ns_mass += volume;
                    ns_moment += volume * ns_trans.GetPack()[i].center;
                    inertia_tensor += 2.f / 5.f * volume * std::powf(ns_trans.GetPack()[i].radius, 2.f) + volume * dot(ns_trans.GetPack()[i].center, ns_trans.GetPack()[i].center);
                }

                ns_packs_data.ns_id = pack_num;
                ns_packs_data.centroid = ns_moment / ns_mass;
                ns_packs_data.mass = ns_mass * 2000.f;
                ns_packs_data.vel = make_float2(0.f);
                ns_packs_data.angle_vel = 0.f;
                ns_packs_data.angle_acc = 0.f;
                ns_packs_data.sub_num = sub_num;

                auto pack_rot = make_rotation2(0.f);
                ns_packs_data.rot = pack_rot.mat;
                ns_packs_data.inertia = inertia_tensor - ns_mass * dot(ns_packs_data.centroid, ns_packs_data.centroid);

                for (size_t i = 0; i < sub_num; i++)
                {

                    ns_packs_data.sub_color_list[i] = ns_trans.GetPack()[i].color;
                    ns_packs_data.sub_radius_list[i] = ns_trans.GetPack()[i].radius;

                    data.min_radius = std::min(ns_trans.GetPack()[i].radius, data.min_radius);
                    data.max_radius = std::max(ns_trans.GetPack()[i].radius, data.max_radius);

                    ns_packs_data.sub_pos_list[i] = inverse_rot2(pack_rot).mat * (ns_trans.GetPack()[i].center - ns_packs_data.centroid);
                    ns_packs_data.sub_rot_list[i] = inverse_rot2(pack_rot).mat * ns_trans.GetPack()[i].rot.mat;

                    ns_packs_data.force_list[i] = make_float2(0.f);
                    ns_packs_data.torque_list[i] = 0.f;

                    ns_mapping map_data;
                    map_data.ns_id = pack_num;
                    map_data.sub_id = i;
                    map_data.rel_pos = ns_packs_data.sub_pos_list[i];
                    map_data.rel_rot = ns_packs_data.sub_rot_list[i];
                    data.map_data.emplace_back(map_data);
                }

                data.ns_data.emplace_back(ns_packs_data);

                ns_gens.emplace_back(ns_gen_i);
                pack_num++;

                break; // break away from the try-loop

            overlap:
                if (tries++ == max_attempt)
                    return;
            }
        }
        return;
    }

    void CudaVolumeEmitter::BuildNsDemVolume(DemNSBoxVolumeData &data,
                                             const std::vector<NSPack> &ns_data)
    {

        data.min_radius = Huge<float>();
        data.max_radius = Tiny<float>();

        for (size_t idx = 0; idx < ns_data.size(); idx++)
        {
            auto ns = ns_data[idx];

            // for clump
            non_spherical_particles ns_packs_data;
            float ns_mass = 0.f;
            float2 ns_moment = make_float2(0.f);
            float inertia_tensor = 0.f;

            int sub_num = ns.GetPack().size();
            for (size_t i = 0; i < sub_num; i++)
            {
                auto cen = ns.GetPack()[i].center;
                auto rad = ns.GetPack()[i].radius;
                data.sphere_data.emplace_back(ns_sphere_data(cen, rad, ns.GetPack()[i].color, i));

                float volume = (4.f / 3.f) * KIRI_PI * std::powf(rad, 3.f);

                ns_mass += volume;
                ns_moment += volume * cen;
                inertia_tensor += 2.f / 5.f * volume * std::powf(rad, 2.f) + volume * dot(cen, cen);
            }

            ns_packs_data.ns_id = idx;
            ns_packs_data.centroid = ns_moment / ns_mass;
            ns_packs_data.mass = ns_mass * 2000.f;
            ns_packs_data.vel = make_float2(0.f);
            ns_packs_data.angle_vel = 0.f;
            ns_packs_data.angle_acc = 0.f;
            ns_packs_data.sub_num = sub_num;

            auto pack_rot = make_rotation2(0.f);
            ns_packs_data.rot = pack_rot.mat;
            ns_packs_data.inertia = inertia_tensor - ns_mass * dot(ns_packs_data.centroid, ns_packs_data.centroid);

            for (size_t i = 0; i < sub_num; i++)
            {

                auto cen = ns.GetPack()[i].center;
                auto rad = ns.GetPack()[i].radius;

                ns_packs_data.sub_color_list[i] = ns.GetPack()[i].color;
                ns_packs_data.sub_radius_list[i] = rad;

                data.min_radius = std::min(rad, data.min_radius);
                data.max_radius = std::max(rad, data.max_radius);

                ns_packs_data.sub_pos_list[i] = inverse_rot2(pack_rot).mat * (cen - ns_packs_data.centroid);
                ns_packs_data.sub_rot_list[i] = inverse_rot2(pack_rot).mat * ns.GetPack()[i].rot.mat;

                ns_packs_data.force_list[i] = make_float2(0.f);
                ns_packs_data.torque_list[i] = 0.f;

                ns_mapping map_data;
                map_data.ns_id = idx;
                map_data.sub_id = i;
                map_data.rel_pos = ns_packs_data.sub_pos_list[i];
                map_data.rel_rot = ns_packs_data.sub_rot_list[i];
                data.map_data.emplace_back(map_data);
            }

            data.ns_data.emplace_back(ns_packs_data);
        }
    }
} // namespace KIRI
