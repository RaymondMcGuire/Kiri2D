/***
 * @Author: Xu.WANG
 * @Date: 2022-01-04 01:33:24
 * @LastEditTime: 2022-01-04 01:33:25
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_TINYOBJ_UTILS_H_
#define _HDV_TINYOBJ_UTILS_H_

#pragma once

#include <tiny_obj_loader.h>
#define TINYOBJLOADER_IMPLEMENTATION

namespace HDV::Voronoi
{
    bool WriteMat(const std::string &fileName, const std::vector<tinyobj::material_t> &materials)
    {
        FILE *fp = fopen(fileName.c_str(), "w");
        if (!fp)
        {
            fprintf(stderr, "Failed to open file [ %s ] for write.\n", fileName.c_str());
            return false;
        }

        for (size_t i = 0; i < materials.size(); i++)
        {

            tinyobj::material_t mat = materials[i];

            fprintf(fp, "newmtl %s\n", mat.name.c_str());
            fprintf(fp, "Ka %f %f %f\n", mat.ambient[0], mat.ambient[1], mat.ambient[2]);
            fprintf(fp, "Kd %f %f %f\n", mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
            fprintf(fp, "Ks %f %f %f\n", mat.specular[0], mat.specular[1], mat.specular[2]);
            fprintf(fp, "Kt %f %f %f\n", mat.transmittance[0], mat.specular[1], mat.specular[2]);
            fprintf(fp, "Ke %f %f %f\n", mat.emission[0], mat.emission[1], mat.emission[2]);
            fprintf(fp, "Ns %f\n", mat.shininess);
            fprintf(fp, "Ni %f\n", mat.ior);
            fprintf(fp, "illum %d\n", mat.illum);
            fprintf(fp, "\n");
            // @todo { texture }
        }

        fclose(fp);

        return true;
    }

    bool tinyObjWriter(const String &fileName, const tinyobj::attrib_t &attributes, const std::vector<tinyobj::shape_t> &shapes, const std::vector<tinyobj::material_t> &materials, bool coordTransform = false)
    {
        String export_path = String(EXPORT_PATH) + "voro/" + fileName + ".obj";

        FILE *fp = fopen(export_path.c_str(), "w");
        if (!fp)
        {
            fprintf(stderr, "Failed to open file [ %s ] for write.\n", export_path.c_str());
            return false;
        }

        std::string base_name = fileName;
        std::string material_filename = base_name + ".mtl";

        int prev_material_id = -1;

        fprintf(fp, "mtllib %s\n\n", material_filename.c_str());

        // vertices
        for (size_t k = 0; k < attributes.vertices.size(); k += 3)
        {
            if (coordTransform)
            {
                fprintf(fp, "v %f %f %f\n",
                        attributes.vertices[k + 0],
                        attributes.vertices[k + 2],
                        -attributes.vertices[k + 1]);
            }
            else
            {
                fprintf(fp, "v %f %f %f\n",
                        attributes.vertices[k + 0],
                        attributes.vertices[k + 1],
                        attributes.vertices[k + 2]);
            }
        }

        fprintf(fp, "\n");

        // normal
        for (size_t k = 0; k < attributes.normals.size(); k += 3)
        {
            if (coordTransform)
            {
                fprintf(fp, "vn %f %f %f\n",
                        attributes.normals[k + 0],
                        attributes.normals[k + 2],
                        -attributes.normals[k + 1]);
            }
            else
            {
                fprintf(fp, "vn %f %f %f\n",
                        attributes.normals[k + 0],
                        attributes.normals[k + 1],
                        attributes.normals[k + 2]);
            }
        }

        fprintf(fp, "\n");

        // texcoord
        for (size_t k = 0; k < attributes.texcoords.size(); k += 2)
        {
            fprintf(fp, "vt %f %f\n",
                    attributes.texcoords[k + 0],
                    attributes.texcoords[k + 1]);
        }

        for (size_t i = 0; i < shapes.size(); i++)
        {
            fprintf(fp, "\n");

            if (shapes[i].name.empty())
            {
                fprintf(fp, "g Unknown\n");
            }
            else
            {
                fprintf(fp, "g %s\n", shapes[i].name.c_str());
            }

            bool has_vn = false;
            bool has_vt = false;
            // Assumes normals and textures are set shape-wise.
            if (shapes[i].mesh.indices.size() > 0)
            {
                has_vn = shapes[i].mesh.indices[0].normal_index != -1;
                has_vt = shapes[i].mesh.indices[0].texcoord_index != -1;
            }

            // face
            int face_index = 0;
            for (size_t k = 0; k < shapes[i].mesh.indices.size(); k += shapes[i].mesh.num_face_vertices[face_index++])
            {
                // Check Materials
                int material_id = shapes[i].mesh.material_ids[face_index];
                if (material_id != prev_material_id)
                {
                    std::string material_name = materials[material_id].name;
                    fprintf(fp, "usemtl %s\n", material_name.c_str());
                    prev_material_id = material_id;
                }

                unsigned char v_per_f = shapes[i].mesh.num_face_vertices[face_index];
                // Imperformant, but if you want to have variable vertices per face, you need some kind of a dynamic loop.
                fprintf(fp, "f");
                for (int l = 0; l < v_per_f; l++)
                {
                    const tinyobj::index_t &ref = shapes[i].mesh.indices[k + l];
                    if (has_vn && has_vt)
                    {
                        // v0/t0/vn0
                        fprintf(fp, " %d/%d/%d", ref.vertex_index + 1, ref.texcoord_index + 1, ref.normal_index + 1);
                        continue;
                    }
                    if (has_vn && !has_vt)
                    {
                        // v0//vn0
                        fprintf(fp, " %d//%d", ref.vertex_index + 1, ref.normal_index + 1);
                        continue;
                    }
                    if (!has_vn && has_vt)
                    {
                        // v0/vt0
                        fprintf(fp, " %d/%d", ref.vertex_index + 1, ref.texcoord_index + 1);
                        continue;
                    }
                    if (!has_vn && !has_vt)
                    {
                        // v0 v1 v2
                        fprintf(fp, " %d", ref.vertex_index + 1);
                        continue;
                    }
                }
                fprintf(fp, "\n");
            }
        }

        fclose(fp);

        //
        // Write material file
        //
        bool ret = WriteMat(material_filename, materials);

        return ret;
    }

} // namespace HDV::Voronoi

#endif /* _HDV_TINYOBJ_UTILS_H_ */