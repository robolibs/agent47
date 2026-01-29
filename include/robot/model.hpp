#pragma once

#include <datapod/adapters.hpp>
#include <datapod/pods/adapters/error.hpp>
#include <datapod/pods/adapters/optional.hpp>
#include <datapod/pods/adapters/result.hpp>
#include <datapod/spatial.hpp>

#include "../robomod/robomod.hpp"

namespace robot {

    enum class JointType : dp::u8 {
        Unknown = 0,
        Revolute,
        Continuous,
        Prismatic,
        Floating,
        Planar,
        Fixed,
    };

    enum class SensorType : dp::u8 {
        Unknown = 0,
        Camera,
        Ray,
    };

    struct Color {
        dp::f32 r = 0.0f;
        dp::f32 g = 0.0f;
        dp::f32 b = 0.0f;
        dp::f32 a = 1.0f;
    };

    struct Material {
        dp::u32 index = 0;
        dp::String name;
        dp::String texture_filename;
        Color color;
    };

    struct Geometry {
        enum class Type : dp::u8 { Unknown = 0, Box, Cylinder, Sphere, Mesh };

        Type type = Type::Unknown;
        dp::Vector<dp::f64> params;
        dp::String mesh_filename;
        dp::mat::Vector<dp::f64, 3> mesh_scale{1.0, 1.0, 1.0};
    };

    struct Visual {
        dp::String name;
        dp::Pose origin;
        dp::Optional<Geometry> geometry;
        dp::u32 material_index = 0;
        dp::String material_name;
    };

    struct Collision {
        dp::String name;
        dp::Pose origin;
        dp::Optional<Geometry> geometry;
    };

    struct Inertial {
        dp::Pose origin;
        dp::f64 mass = 0.0;
        dp::f64 ixx = 0.0;
        dp::f64 ixy = 0.0;
        dp::f64 ixz = 0.0;
        dp::f64 iyy = 0.0;
        dp::f64 iyz = 0.0;
        dp::f64 izz = 0.0;
    };

    struct SensorCamera {
        dp::u32 width = 0;
        dp::u32 height = 0;
        dp::String format;
        dp::f64 hfov = 0.0;
        dp::f64 near = 0.0;
        dp::f64 far = 0.0;
    };

    struct SensorRay {
        dp::u32 horizontal_samples = 0;
        dp::f64 horizontal_resolution = 0.0;
        dp::f64 horizontal_min_angle = 0.0;
        dp::f64 horizontal_max_angle = 0.0;

        dp::u32 vertical_samples = 0;
        dp::f64 vertical_resolution = 0.0;
        dp::f64 vertical_min_angle = 0.0;
        dp::f64 vertical_max_angle = 0.0;
    };

    struct Sensor {
        dp::u32 index = 0;
        dp::u32 parent_link_index = 0;

        dp::String name;
        SensorType type = SensorType::Unknown;
        dp::Pose origin;
        dp::String raw_xml;

        dp::Optional<SensorCamera> camera;
        dp::Optional<SensorRay> ray;
    };

    struct Link {
        dp::u32 index = 0;
        dp::String name;
        dp::Optional<Inertial> inertial;
        dp::Vector<Visual> visuals;
        dp::Vector<Collision> collisions;

        dp::Vector<dp::u32> sensor_indices;
    };

    struct JointLimits {
        dp::f64 lower = 0.0;
        dp::f64 upper = 0.0;
        dp::f64 effort = 0.0;
        dp::f64 velocity = 0.0;
    };

    struct JointDynamics {
        dp::f64 damping = 0.0;
        dp::f64 friction = 0.0;
    };

    struct Joint {
        dp::u32 index = 0;
        dp::String name;
        JointType type = JointType::Unknown;

        dp::u32 parent_link_index = 0;
        dp::u32 child_link_index = 0;
        dp::String parent_link;
        dp::String child_link;

        dp::Pose parent_to_joint;
        dp::mat::Vector<dp::f64, 3> axis{1.0, 0.0, 0.0};

        dp::Optional<JointLimits> limits;
        dp::Optional<JointDynamics> dynamics;
    };

    struct Model {
        dp::String name;
        dp::String root_link;

        dp::Vector<Link> links;
        dp::Vector<Joint> joints;
        dp::Vector<Material> materials;
        dp::Vector<Sensor> sensors;

        static dp::Result<Model> from_urdf_string(const dp::String &xml);
    };

    inline JointType to_joint_type(robomod::JointType t) {
        switch (t) {
        case robomod::JointType::Revolute:
            return JointType::Revolute;
        case robomod::JointType::Continuous:
            return JointType::Continuous;
        case robomod::JointType::Prismatic:
            return JointType::Prismatic;
        case robomod::JointType::Floating:
            return JointType::Floating;
        case robomod::JointType::Planar:
            return JointType::Planar;
        case robomod::JointType::Fixed:
            return JointType::Fixed;
        default:
            return JointType::Unknown;
        }
    }

    inline SensorType to_sensor_type(robomod::SensorType t) {
        switch (t) {
        case robomod::SensorType::Camera:
            return SensorType::Camera;
        case robomod::SensorType::Ray:
            return SensorType::Ray;
        default:
            return SensorType::Unknown;
        }
    }

    inline Geometry::Type to_geom_type(robomod::Geometry::Type t) {
        switch (t) {
        case robomod::Geometry::Type::Box:
            return Geometry::Type::Box;
        case robomod::Geometry::Type::Cylinder:
            return Geometry::Type::Cylinder;
        case robomod::Geometry::Type::Sphere:
            return Geometry::Type::Sphere;
        case robomod::Geometry::Type::Mesh:
            return Geometry::Type::Mesh;
        default:
            return Geometry::Type::Unknown;
        }
    }

    inline dp::Result<Model> Model::from_urdf_string(const dp::String &xml) {
        auto parsed = robomod::Model::from_urdf_string(xml);
        if (parsed.is_err()) {
            return dp::Result<Model>::err(parsed.error());
        }
        const auto &in = parsed.value();

        Model out;
        out.name = in.name;
        out.root_link = in.root_link;

        // Materials
        out.materials.reserve(in.materials.size());
        for (dp::usize i = 0; i < in.materials.size(); ++i) {
            const auto &m = in.materials[i];
            Material mm;
            mm.index = static_cast<dp::u32>(i);
            mm.name = m.name;
            mm.texture_filename = m.texture_filename;
            mm.color.r = m.color.r;
            mm.color.g = m.color.g;
            mm.color.b = m.color.b;
            mm.color.a = m.color.a;
            out.materials.push_back(mm);
        }

        // Links
        out.links.reserve(in.links.size());
        for (dp::usize i = 0; i < in.links.size(); ++i) {
            const auto &l = in.links[i];
            Link ll;
            ll.index = static_cast<dp::u32>(i);
            ll.name = l.name;

            if (l.inertial.has_value()) {
                const auto &ii = *l.inertial;
                Inertial inert;
                inert.origin = ii.origin;
                inert.mass = ii.mass;
                inert.ixx = ii.ixx;
                inert.ixy = ii.ixy;
                inert.ixz = ii.ixz;
                inert.iyy = ii.iyy;
                inert.iyz = ii.iyz;
                inert.izz = ii.izz;
                ll.inertial = inert;
            }

            ll.visuals.reserve(l.visuals.size());
            for (const auto &v : l.visuals) {
                Visual vv;
                vv.name = v.name;
                vv.origin = v.origin;
                vv.material_name = v.material_name;
                if (v.geometry.has_value()) {
                    const auto &g = *v.geometry;
                    Geometry gg;
                    gg.type = to_geom_type(g.type);
                    gg.params = g.params;
                    gg.mesh_filename = g.mesh_filename;
                    gg.mesh_scale = g.mesh_scale;
                    vv.geometry = gg;
                }
                // Resolve material index by name.
                if (!vv.material_name.empty()) {
                    for (const auto &mat : out.materials) {
                        if (mat.name == vv.material_name) {
                            vv.material_index = mat.index;
                            break;
                        }
                    }
                }
                ll.visuals.push_back(vv);
            }

            ll.collisions.reserve(l.collisions.size());
            for (const auto &c : l.collisions) {
                Collision cc;
                cc.name = c.name;
                cc.origin = c.origin;
                if (c.geometry.has_value()) {
                    const auto &g = *c.geometry;
                    Geometry gg;
                    gg.type = to_geom_type(g.type);
                    gg.params = g.params;
                    gg.mesh_filename = g.mesh_filename;
                    gg.mesh_scale = g.mesh_scale;
                    cc.geometry = gg;
                }
                ll.collisions.push_back(cc);
            }

            out.links.push_back(ll);
        }

        // Helper: link name -> index.
        auto link_index_of = [&](const dp::String &name) -> dp::Optional<dp::u32> {
            for (const auto &l : out.links) {
                if (l.name == name) {
                    return l.index;
                }
            }
            return dp::Optional<dp::u32>{};
        };

        // Joints
        out.joints.reserve(in.joints.size());
        for (dp::usize i = 0; i < in.joints.size(); ++i) {
            const auto &j = in.joints[i];
            Joint jj;
            jj.index = static_cast<dp::u32>(i);
            jj.name = j.name;
            jj.type = to_joint_type(j.type);
            jj.parent_link = j.parent_link;
            jj.child_link = j.child_link;
            jj.parent_to_joint = j.parent_to_joint;
            jj.axis = j.axis;

            auto pidx = link_index_of(jj.parent_link);
            auto cidx = link_index_of(jj.child_link);
            if (!pidx.has_value() || !cidx.has_value()) {
                return dp::Result<Model>::err(dp::Error::invalid_argument("joint references missing link"));
            }
            jj.parent_link_index = *pidx;
            jj.child_link_index = *cidx;

            if (j.limits.has_value()) {
                JointLimits lim;
                lim.lower = j.limits->lower;
                lim.upper = j.limits->upper;
                lim.effort = j.limits->effort;
                lim.velocity = j.limits->velocity;
                jj.limits = lim;
            }
            if (j.dynamics.has_value()) {
                JointDynamics dyn;
                dyn.damping = j.dynamics->damping;
                dyn.friction = j.dynamics->friction;
                jj.dynamics = dyn;
            }

            out.joints.push_back(jj);
        }

        // Sensors: flatten per-link sensors into a global vector and store indices in links.
        for (auto &l : out.links) {
            l.sensor_indices.clear();
        }
        for (const auto &l_in : in.links) {
            auto lidx = link_index_of(l_in.name);
            if (!lidx.has_value()) {
                continue;
            }
            for (const auto &s : l_in.sensors) {
                Sensor ss;
                ss.index = static_cast<dp::u32>(out.sensors.size());
                ss.parent_link_index = *lidx;
                ss.name = s.name;
                ss.type = to_sensor_type(s.type);
                ss.origin = s.origin;
                ss.raw_xml = s.raw_xml;
                if (s.camera.has_value()) {
                    SensorCamera c;
                    c.width = s.camera->width;
                    c.height = s.camera->height;
                    c.format = s.camera->format;
                    c.hfov = s.camera->hfov;
                    c.near = s.camera->near;
                    c.far = s.camera->far;
                    ss.camera = c;
                }
                if (s.ray.has_value()) {
                    SensorRay r;
                    r.horizontal_samples = s.ray->horizontal_samples;
                    r.horizontal_resolution = s.ray->horizontal_resolution;
                    r.horizontal_min_angle = s.ray->horizontal_min_angle;
                    r.horizontal_max_angle = s.ray->horizontal_max_angle;
                    r.vertical_samples = s.ray->vertical_samples;
                    r.vertical_resolution = s.ray->vertical_resolution;
                    r.vertical_min_angle = s.ray->vertical_min_angle;
                    r.vertical_max_angle = s.ray->vertical_max_angle;
                    ss.ray = r;
                }

                out.sensors.push_back(ss);
                out.links[*lidx].sensor_indices.push_back(ss.index);
            }
        }

        return dp::Result<Model>::ok(out);
    }

} // namespace robot
