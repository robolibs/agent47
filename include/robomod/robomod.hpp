#pragma once

// Header-only URDF parser + data model.
//
// This is a dp-native, header-only port of the small TinyXML-based URDF_Parser
// vendored under xtra/URDF_Parser, with a few robustness ideas borrowed from
// urdfdom (multiple visuals/collisions, root-link detection).
//
// Notes:
// - All types live in namespace robomod (no nested namespaces).
// - Uses datapod containers and error handling:
//   dp::String, dp::Vector, dp::Optional, dp::Result
// - Parsing is done via the vendored TinyXML (xtra/URDF_Parser/include/tinyxml/txml.h)
//   and is entirely header-only.

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <memory>

#include <datapod/adapters.hpp>
#include <datapod/pods/adapters/error.hpp>
#include <datapod/pods/adapters/optional.hpp>
#include <datapod/pods/adapters/result.hpp>
#include <datapod/spatial.hpp>

#include "tinyxml/txml.h"

namespace robomod {

    // ---------------------------------------------------------------------------------------------
    // Small helpers
    // ---------------------------------------------------------------------------------------------

    static inline dp::String dp_string(const char *cstr) {
        if (!cstr) {
            return dp::String("");
        }
        return dp::String(cstr);
    }

    static inline dp::Result<dp::f64> parse_f64(const char *s) {
        if (!s) {
            return dp::Result<dp::f64>::err(dp::Error::invalid_argument("missing numeric attribute"));
        }
        char *end = nullptr;
        const double v = std::strtod(s, &end);
        if (end == s || (end && *end != '\0')) {
            return dp::Result<dp::f64>::err(dp::Error::invalid_argument("invalid numeric attribute"));
        }
        return dp::Result<dp::f64>::ok(static_cast<dp::f64>(v));
    }

    static inline dp::Result<dp::Vector<dp::f64>> parse_f64_list(const char *s, dp::usize expected) {
        if (!s) {
            return dp::Result<dp::Vector<dp::f64>>::err(dp::Error::invalid_argument("missing numeric list"));
        }

        dp::Vector<dp::f64> out;
        out.reserve(expected);

        const char *p = s;
        while (*p != '\0') {
            while (std::isspace(static_cast<unsigned char>(*p))) {
                ++p;
            }
            if (*p == '\0') {
                break;
            }
            char *end = nullptr;
            const double v = std::strtod(p, &end);
            if (end == p) {
                return dp::Result<dp::Vector<dp::f64>>::err(dp::Error::invalid_argument("invalid numeric list"));
            }
            out.push_back(static_cast<dp::f64>(v));
            p = end;
        }

        if (out.size() != expected) {
            return dp::Result<dp::Vector<dp::f64>>::err(dp::Error::invalid_argument("wrong numeric list arity"));
        }

        return dp::Result<dp::Vector<dp::f64>>::ok(out);
    }

    // rpy is ZYX (roll, pitch, yaw) as in URDF.
    static inline dp::Quaternion quat_from_rpy(dp::f64 roll, dp::f64 pitch, dp::f64 yaw) {
        return dp::Quaternion::from_euler(roll, pitch, yaw);
    }

    // ---------------------------------------------------------------------------------------------
    // Data model
    // ---------------------------------------------------------------------------------------------

    enum class JointType : dp::u8 {
        Unknown = 0,
        Revolute,
        Continuous,
        Prismatic,
        Floating,
        Planar,
        Fixed,
    };

    struct Color {
        dp::f32 r = 0.0f;
        dp::f32 g = 0.0f;
        dp::f32 b = 0.0f;
        dp::f32 a = 1.0f;
    };

    struct Material {
        dp::String name;
        dp::String texture_filename;
        Color color;
    };

    struct Geometry {
        enum class Type : dp::u8 { Unknown = 0, Box, Cylinder, Sphere, Mesh };

        Type type = Type::Unknown;
        dp::Vector<dp::f64> params; // Box: [x y z], Cylinder: [radius length], Sphere: [radius]
        dp::String mesh_filename;
        dp::mat::Vector<dp::f64, 3> mesh_scale{1.0, 1.0, 1.0};
    };

    struct Visual {
        dp::String name;
        dp::Pose origin;
        dp::Optional<Geometry> geometry;
        dp::String material_name;
        dp::Optional<Material> material;
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

    struct Link {
        dp::String name;
        dp::Optional<Inertial> inertial;
        dp::Vector<Visual> visuals;
        dp::Vector<Collision> collisions;

        // Sensors are not part of core URDF, but commonly provided via <sensor>
        // blocks (e.g. Gazebo). We parse a small, self-contained subset based on
        // urdfdom's urdf_sensor.cpp (camera/ray) and also keep raw XML payload.
        dp::Vector<struct Sensor> sensors;
    };

    enum class SensorType : dp::u8 {
        Unknown = 0,
        Camera,
        Ray,
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
        dp::String name;
        SensorType type = SensorType::Unknown;
        dp::Pose origin;
        dp::String raw_xml;

        dp::Optional<SensorCamera> camera;
        dp::Optional<SensorRay> ray;
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
        dp::String name;
        JointType type = JointType::Unknown;
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

        static dp::Result<Model> from_urdf_string(const dp::String &xml);
    };

    // ---------------------------------------------------------------------------------------------
    // Parsing
    // ---------------------------------------------------------------------------------------------

    static inline dp::Result<dp::Pose> parse_origin(TiXmlElement *origin_xml) {
        dp::Pose out;
        out.point = dp::Point{0.0, 0.0, 0.0};
        out.rotation = dp::Quaternion{1.0, 0.0, 0.0, 0.0};

        if (!origin_xml) {
            return dp::Result<dp::Pose>::ok(out);
        }

        if (const char *xyz = origin_xml->Attribute("xyz")) {
            auto vals = parse_f64_list(xyz, 3);
            if (vals.is_err()) {
                return dp::Result<dp::Pose>::err(vals.error());
            }
            out.point.x = vals.value()[0];
            out.point.y = vals.value()[1];
            out.point.z = vals.value()[2];
        }

        if (const char *rpy = origin_xml->Attribute("rpy")) {
            auto vals = parse_f64_list(rpy, 3);
            if (vals.is_err()) {
                return dp::Result<dp::Pose>::err(vals.error());
            }
            out.rotation = quat_from_rpy(vals.value()[0], vals.value()[1], vals.value()[2]);
        }

        return dp::Result<dp::Pose>::ok(out);
    }

    static inline dp::Result<Geometry> parse_geometry(TiXmlElement *geom_xml) {
        if (!geom_xml) {
            return dp::Result<Geometry>::err(dp::Error::invalid_argument("missing geometry"));
        }

        TiXmlElement *shape = geom_xml->FirstChildElement();
        if (!shape) {
            return dp::Result<Geometry>::err(dp::Error::invalid_argument("geometry has no shape"));
        }

        Geometry g;
        const dp::String tag = dp_string(shape->Value());

        if (tag == "box") {
            g.type = Geometry::Type::Box;
            auto vals = parse_f64_list(shape->Attribute("size"), 3);
            if (vals.is_err()) {
                return dp::Result<Geometry>::err(vals.error());
            }
            g.params = vals.value();
            return dp::Result<Geometry>::ok(g);
        }
        if (tag == "cylinder") {
            g.type = Geometry::Type::Cylinder;
            auto r = parse_f64(shape->Attribute("radius"));
            auto l = parse_f64(shape->Attribute("length"));
            if (r.is_err() || l.is_err()) {
                return dp::Result<Geometry>::err(dp::Error::invalid_argument("invalid cylinder"));
            }
            g.params = dp::Vector<dp::f64>{r.value(), l.value()};
            return dp::Result<Geometry>::ok(g);
        }
        if (tag == "sphere") {
            g.type = Geometry::Type::Sphere;
            auto r = parse_f64(shape->Attribute("radius"));
            if (r.is_err()) {
                return dp::Result<Geometry>::err(r.error());
            }
            g.params = dp::Vector<dp::f64>{r.value()};
            return dp::Result<Geometry>::ok(g);
        }
        if (tag == "mesh") {
            g.type = Geometry::Type::Mesh;
            g.mesh_filename = dp_string(shape->Attribute("filename"));
            if (g.mesh_filename.empty()) {
                return dp::Result<Geometry>::err(dp::Error::invalid_argument("mesh missing filename"));
            }
            if (const char *scale = shape->Attribute("scale")) {
                auto vals = parse_f64_list(scale, 3);
                if (vals.is_err()) {
                    return dp::Result<Geometry>::err(vals.error());
                }
                g.mesh_scale[0] = vals.value()[0];
                g.mesh_scale[1] = vals.value()[1];
                g.mesh_scale[2] = vals.value()[2];
            }
            return dp::Result<Geometry>::ok(g);
        }

        return dp::Result<Geometry>::err(dp::Error::invalid_argument("unknown geometry type"));
    }

    static inline dp::Result<Material> parse_material(TiXmlElement *mat_xml, bool only_name_ok) {
        if (!mat_xml) {
            return dp::Result<Material>::err(dp::Error::invalid_argument("missing material"));
        }
        Material m;
        m.name = dp_string(mat_xml->Attribute("name"));
        if (m.name.empty()) {
            return dp::Result<Material>::err(dp::Error::invalid_argument("material missing name"));
        }

        bool has_color = false;
        bool has_texture = false;

        if (TiXmlElement *t = mat_xml->FirstChildElement("texture")) {
            m.texture_filename = dp_string(t->Attribute("filename"));
            has_texture = !m.texture_filename.empty();
        }

        if (TiXmlElement *c = mat_xml->FirstChildElement("color")) {
            if (const char *rgba = c->Attribute("rgba")) {
                auto vals = parse_f64_list(rgba, 4);
                if (vals.is_ok()) {
                    m.color.r = static_cast<float>(vals.value()[0]);
                    m.color.g = static_cast<float>(vals.value()[1]);
                    m.color.b = static_cast<float>(vals.value()[2]);
                    m.color.a = static_cast<float>(vals.value()[3]);
                    has_color = true;
                }
            }
        }

        if (!only_name_ok && !has_color && !has_texture) {
            return dp::Result<Material>::err(dp::Error::invalid_argument("material not fully defined"));
        }
        return dp::Result<Material>::ok(m);
    }

    static inline dp::Result<Visual> parse_visual(TiXmlElement *vis_xml) {
        Visual v;
        if (!vis_xml) {
            return dp::Result<Visual>::err(dp::Error::invalid_argument("missing visual"));
        }
        v.name = dp_string(vis_xml->Attribute("name"));

        auto origin = parse_origin(vis_xml->FirstChildElement("origin"));
        if (origin.is_err()) {
            return dp::Result<Visual>::err(origin.error());
        }
        v.origin = origin.value();

        auto geom = parse_geometry(vis_xml->FirstChildElement("geometry"));
        if (geom.is_err()) {
            return dp::Result<Visual>::err(geom.error());
        }
        v.geometry = geom.value();

        if (TiXmlElement *mat = vis_xml->FirstChildElement("material")) {
            v.material_name = dp_string(mat->Attribute("name"));
            auto parsed = parse_material(mat, /*only_name_ok=*/true);
            if (parsed.is_ok()) {
                v.material = parsed.value();
            }
        }
        return dp::Result<Visual>::ok(v);
    }

    static inline dp::Result<Collision> parse_collision(TiXmlElement *col_xml) {
        Collision c;
        if (!col_xml) {
            return dp::Result<Collision>::err(dp::Error::invalid_argument("missing collision"));
        }
        c.name = dp_string(col_xml->Attribute("name"));

        auto origin = parse_origin(col_xml->FirstChildElement("origin"));
        if (origin.is_err()) {
            return dp::Result<Collision>::err(origin.error());
        }
        c.origin = origin.value();

        auto geom = parse_geometry(col_xml->FirstChildElement("geometry"));
        if (geom.is_err()) {
            return dp::Result<Collision>::err(geom.error());
        }
        c.geometry = geom.value();

        return dp::Result<Collision>::ok(c);
    }

    static inline dp::Result<Inertial> parse_inertial(TiXmlElement *inertial_xml) {
        if (!inertial_xml) {
            return dp::Result<Inertial>::err(dp::Error::invalid_argument("missing inertial"));
        }
        Inertial i;
        auto origin = parse_origin(inertial_xml->FirstChildElement("origin"));
        if (origin.is_err()) {
            return dp::Result<Inertial>::err(origin.error());
        }
        i.origin = origin.value();

        TiXmlElement *mass_xml = inertial_xml->FirstChildElement("mass");
        if (!mass_xml) {
            return dp::Result<Inertial>::err(dp::Error::invalid_argument("inertial missing mass"));
        }
        auto mass = parse_f64(mass_xml->Attribute("value"));
        if (mass.is_err()) {
            return dp::Result<Inertial>::err(mass.error());
        }
        i.mass = mass.value();

        TiXmlElement *inertia_xml = inertial_xml->FirstChildElement("inertia");
        if (!inertia_xml) {
            return dp::Result<Inertial>::err(dp::Error::invalid_argument("inertial missing inertia"));
        }

        auto ixx = parse_f64(inertia_xml->Attribute("ixx"));
        auto ixy = parse_f64(inertia_xml->Attribute("ixy"));
        auto ixz = parse_f64(inertia_xml->Attribute("ixz"));
        auto iyy = parse_f64(inertia_xml->Attribute("iyy"));
        auto iyz = parse_f64(inertia_xml->Attribute("iyz"));
        auto izz = parse_f64(inertia_xml->Attribute("izz"));
        if (ixx.is_err() || ixy.is_err() || ixz.is_err() || iyy.is_err() || iyz.is_err() || izz.is_err()) {
            return dp::Result<Inertial>::err(dp::Error::invalid_argument("invalid inertia"));
        }

        i.ixx = ixx.value();
        i.ixy = ixy.value();
        i.ixz = ixz.value();
        i.iyy = iyy.value();
        i.iyz = iyz.value();
        i.izz = izz.value();

        return dp::Result<Inertial>::ok(i);
    }

    static inline JointType joint_type_from_string(const dp::String &s) {
        if (s == "revolute") {
            return JointType::Revolute;
        }
        if (s == "continuous") {
            return JointType::Continuous;
        }
        if (s == "prismatic") {
            return JointType::Prismatic;
        }
        if (s == "floating") {
            return JointType::Floating;
        }
        if (s == "planar") {
            return JointType::Planar;
        }
        if (s == "fixed") {
            return JointType::Fixed;
        }
        return JointType::Unknown;
    }

    static inline dp::Result<Joint> parse_joint(TiXmlElement *joint_xml) {
        if (!joint_xml) {
            return dp::Result<Joint>::err(dp::Error::invalid_argument("missing joint"));
        }

        Joint j;
        j.name = dp_string(joint_xml->Attribute("name"));
        if (j.name.empty()) {
            return dp::Result<Joint>::err(dp::Error::invalid_argument("joint missing name"));
        }
        j.type = joint_type_from_string(dp_string(joint_xml->Attribute("type")));

        if (TiXmlElement *parent = joint_xml->FirstChildElement("parent")) {
            j.parent_link = dp_string(parent->Attribute("link"));
        }
        if (TiXmlElement *child = joint_xml->FirstChildElement("child")) {
            j.child_link = dp_string(child->Attribute("link"));
        }
        if (j.parent_link.empty() || j.child_link.empty()) {
            return dp::Result<Joint>::err(dp::Error::invalid_argument("joint missing parent/child"));
        }

        auto origin = parse_origin(joint_xml->FirstChildElement("origin"));
        if (origin.is_err()) {
            return dp::Result<Joint>::err(origin.error());
        }
        j.parent_to_joint = origin.value();

        if (TiXmlElement *axis = joint_xml->FirstChildElement("axis")) {
            if (const char *xyz = axis->Attribute("xyz")) {
                auto vals = parse_f64_list(xyz, 3);
                if (vals.is_ok()) {
                    j.axis[0] = vals.value()[0];
                    j.axis[1] = vals.value()[1];
                    j.axis[2] = vals.value()[2];
                }
            }
        }

        if (TiXmlElement *limits = joint_xml->FirstChildElement("limit")) {
            JointLimits l;
            if (const char *lower = limits->Attribute("lower")) {
                auto v = parse_f64(lower);
                if (v.is_ok()) {
                    l.lower = v.value();
                }
            }
            if (const char *upper = limits->Attribute("upper")) {
                auto v = parse_f64(upper);
                if (v.is_ok()) {
                    l.upper = v.value();
                }
            }
            if (const char *effort = limits->Attribute("effort")) {
                auto v = parse_f64(effort);
                if (v.is_ok()) {
                    l.effort = v.value();
                }
            }
            if (const char *velocity = limits->Attribute("velocity")) {
                auto v = parse_f64(velocity);
                if (v.is_ok()) {
                    l.velocity = v.value();
                }
            }
            j.limits = l;
        }

        if (TiXmlElement *dyn = joint_xml->FirstChildElement("dynamics")) {
            JointDynamics d;
            if (const char *damping = dyn->Attribute("damping")) {
                auto v = parse_f64(damping);
                if (v.is_ok()) {
                    d.damping = v.value();
                }
            }
            if (const char *friction = dyn->Attribute("friction")) {
                auto v = parse_f64(friction);
                if (v.is_ok()) {
                    d.friction = v.value();
                }
            }
            j.dynamics = d;
        }

        return dp::Result<Joint>::ok(j);
    }

    static inline dp::Result<Link> parse_link(TiXmlElement *link_xml) {
        if (!link_xml) {
            return dp::Result<Link>::err(dp::Error::invalid_argument("missing link"));
        }

        Link l;
        l.name = dp_string(link_xml->Attribute("name"));
        if (l.name.empty()) {
            return dp::Result<Link>::err(dp::Error::invalid_argument("link missing name"));
        }

        if (TiXmlElement *inertial = link_xml->FirstChildElement("inertial")) {
            auto parsed = parse_inertial(inertial);
            if (parsed.is_err()) {
                return dp::Result<Link>::err(parsed.error());
            }
            l.inertial = parsed.value();
        }

        for (TiXmlElement *vis = link_xml->FirstChildElement("visual"); vis; vis = vis->NextSiblingElement("visual")) {
            auto parsed = parse_visual(vis);
            if (parsed.is_err()) {
                return dp::Result<Link>::err(parsed.error());
            }
            l.visuals.push_back(parsed.value());
        }

        for (TiXmlElement *col = link_xml->FirstChildElement("collision"); col;
             col = col->NextSiblingElement("collision")) {
            auto parsed = parse_collision(col);
            if (parsed.is_err()) {
                return dp::Result<Link>::err(parsed.error());
            }
            l.collisions.push_back(parsed.value());
        }

        // Non-standard but common: <sensor> blocks directly under <link>.
        // Supports a self-contained subset: <sensor><camera>...</camera></sensor> and
        // <sensor><ray>...</ray></sensor>.
        for (TiXmlElement *sensor = link_xml->FirstChildElement("sensor"); sensor;
             sensor = sensor->NextSiblingElement("sensor")) {
            // We'll parse known shapes and also keep raw xml.
            Sensor s;
            s.name = dp_string(sensor->Attribute("name"));
            auto origin = parse_origin(sensor->FirstChildElement("origin"));
            if (origin.is_ok()) {
                s.origin = origin.value();
            }

            // Store raw xml for forward compatibility.
            {
                TiXmlPrinter printer;
                sensor->Accept(&printer);
                s.raw_xml = dp_string(printer.CStr());
            }

            if (TiXmlElement *camera = sensor->FirstChildElement("camera")) {
                SensorCamera cam;
                TiXmlElement *image = camera->FirstChildElement("image");
                if (image) {
                    const char *width = image->Attribute("width");
                    const char *height = image->Attribute("height");
                    const char *format = image->Attribute("format");
                    const char *hfov = image->Attribute("hfov");
                    const char *near = image->Attribute("near");
                    const char *far = image->Attribute("far");
                    if (width && height && format && hfov && near && far) {
                        cam.width = static_cast<dp::u32>(std::strtoul(width, nullptr, 10));
                        cam.height = static_cast<dp::u32>(std::strtoul(height, nullptr, 10));
                        cam.format = dp_string(format);
                        auto hf = parse_f64(hfov);
                        auto n = parse_f64(near);
                        auto f = parse_f64(far);
                        if (hf.is_ok() && n.is_ok() && f.is_ok()) {
                            cam.hfov = hf.value();
                            cam.near = n.value();
                            cam.far = f.value();
                            s.type = SensorType::Camera;
                            s.camera = cam;
                        }
                    }
                }
            } else if (TiXmlElement *ray = sensor->FirstChildElement("ray")) {
                SensorRay r;
                if (TiXmlElement *h = ray->FirstChildElement("horizontal")) {
                    if (const char *samples = h->Attribute("samples")) {
                        r.horizontal_samples = static_cast<dp::u32>(std::strtoul(samples, nullptr, 10));
                    }
                    if (const char *resolution = h->Attribute("resolution")) {
                        auto v = parse_f64(resolution);
                        if (v.is_ok()) {
                            r.horizontal_resolution = v.value();
                        }
                    }
                    if (const char *min_angle = h->Attribute("min_angle")) {
                        auto v = parse_f64(min_angle);
                        if (v.is_ok()) {
                            r.horizontal_min_angle = v.value();
                        }
                    }
                    if (const char *max_angle = h->Attribute("max_angle")) {
                        auto v = parse_f64(max_angle);
                        if (v.is_ok()) {
                            r.horizontal_max_angle = v.value();
                        }
                    }
                }
                if (TiXmlElement *v = ray->FirstChildElement("vertical")) {
                    if (const char *samples = v->Attribute("samples")) {
                        r.vertical_samples = static_cast<dp::u32>(std::strtoul(samples, nullptr, 10));
                    }
                    if (const char *resolution = v->Attribute("resolution")) {
                        auto vv = parse_f64(resolution);
                        if (vv.is_ok()) {
                            r.vertical_resolution = vv.value();
                        }
                    }
                    if (const char *min_angle = v->Attribute("min_angle")) {
                        auto vv = parse_f64(min_angle);
                        if (vv.is_ok()) {
                            r.vertical_min_angle = vv.value();
                        }
                    }
                    if (const char *max_angle = v->Attribute("max_angle")) {
                        auto vv = parse_f64(max_angle);
                        if (vv.is_ok()) {
                            r.vertical_max_angle = vv.value();
                        }
                    }
                }
                s.type = SensorType::Ray;
                s.ray = r;
            }

            l.sensors.push_back(s);
        }

        return dp::Result<Link>::ok(l);
    }

    static inline void apply_materials(Model &m) {
        // Assign named materials to visuals; allow inline (visual-local) materials.
        for (auto &link : m.links) {
            for (auto &vis : link.visuals) {
                if (vis.material.has_value()) {
                    bool found = false;
                    for (const auto &global : m.materials) {
                        if (global.name == vis.material->name) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        m.materials.push_back(*vis.material);
                    }
                }

                if (!vis.material_name.empty()) {
                    for (const auto &global : m.materials) {
                        if (global.name == vis.material_name) {
                            vis.material = global;
                            break;
                        }
                    }
                }
            }
        }
    }

    static inline dp::Result<dp::String> find_root_link(const Model &m) {
        // Root link = link that is never a child in any joint.
        dp::Vector<dp::String> children;
        children.reserve(m.joints.size());
        for (const auto &j : m.joints) {
            children.push_back(j.child_link);
        }

        for (const auto &l : m.links) {
            bool is_child = false;
            for (const auto &c : children) {
                if (c == l.name) {
                    is_child = true;
                    break;
                }
            }
            if (!is_child) {
                return dp::Result<dp::String>::ok(l.name);
            }
        }

        return dp::Result<dp::String>::err(dp::Error::invalid_argument("could not determine root link"));
    }

    inline dp::Result<Model> Model::from_urdf_string(const dp::String &xml) {
        TiXmlDocument doc;
        doc.Parse(xml.c_str());
        if (doc.Error()) {
            return dp::Result<Model>::err(dp::Error::invalid_argument("invalid xml"));
        }

        TiXmlElement *robot = doc.FirstChildElement("robot");
        if (!robot) {
            return dp::Result<Model>::err(dp::Error::invalid_argument("missing <robot>"));
        }

        Model out;
        out.name = dp_string(robot->Attribute("name"));
        if (out.name.empty()) {
            return dp::Result<Model>::err(dp::Error::invalid_argument("robot missing name"));
        }

        for (TiXmlElement *mat = robot->FirstChildElement("material"); mat; mat = mat->NextSiblingElement("material")) {
            auto parsed = parse_material(mat, /*only_name_ok=*/false);
            if (parsed.is_ok()) {
                out.materials.push_back(parsed.value());
            }
        }

        for (TiXmlElement *link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
            auto parsed = parse_link(link);
            if (parsed.is_err()) {
                return dp::Result<Model>::err(parsed.error());
            }
            out.links.push_back(parsed.value());
        }
        if (out.links.empty()) {
            return dp::Result<Model>::err(dp::Error::invalid_argument("no links"));
        }

        for (TiXmlElement *joint = robot->FirstChildElement("joint"); joint;
             joint = joint->NextSiblingElement("joint")) {
            auto parsed = parse_joint(joint);
            if (parsed.is_err()) {
                return dp::Result<Model>::err(parsed.error());
            }
            out.joints.push_back(parsed.value());
        }

        apply_materials(out);

        auto root = find_root_link(out);
        if (root.is_err()) {
            return dp::Result<Model>::err(root.error());
        }
        out.root_link = root.value();

        return dp::Result<Model>::ok(out);
    }

} // namespace robomod
