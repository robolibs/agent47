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

#include <datapod/pods/spatial/robot/model.hpp>

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

    static dp::Result<datapod::robot::Model> from_urdf_string(const dp::String &xml);

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

        return dp::Result<Link>::ok(l);
    }

    static inline dp::Result<dp::String> find_root_link(const dp::Vector<Link> &links,
                                                        const dp::Vector<Joint> &joints) {
        // Root link = link that is never a child in any joint.
        dp::Vector<dp::String> children;
        children.reserve(joints.size());
        for (const auto &j : joints) {
            children.push_back(j.child_link);
        }

        for (const auto &l : links) {
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

    static inline datapod::robot::Joint::Type to_dp_joint_type(JointType t) {
        using JT = datapod::robot::Joint::Type;
        switch (t) {
        case JointType::Revolute:
            return JT::Revolute;
        case JointType::Continuous:
            return JT::Continuous;
        case JointType::Prismatic:
            return JT::Prismatic;
        case JointType::Floating:
            return JT::Floating;
        case JointType::Planar:
            return JT::Planar;
        case JointType::Fixed:
            return JT::Fixed;
        case JointType::Unknown:
        default:
            return JT::Fixed;
        }
    }

    static inline dp::Optional<datapod::robot::Joint::Limits> to_dp_joint_limits(const dp::Optional<JointLimits> &l) {
        if (!l.has_value()) {
            return dp::nullopt;
        }
        datapod::robot::Joint::Limits out;
        out.lower = l->lower;
        out.upper = l->upper;
        out.effort = l->effort;
        out.velocity = l->velocity;
        return out;
    }

    static inline dp::Optional<datapod::robot::Joint::Dynamics>
    to_dp_joint_dynamics(const dp::Optional<JointDynamics> &d) {
        if (!d.has_value()) {
            return dp::nullopt;
        }
        datapod::robot::Joint::Dynamics out;
        out.damping = d->damping;
        out.friction = d->friction;
        return out;
    }

    static inline dp::Optional<datapod::robot::Geometry> to_dp_geometry(const dp::Optional<Geometry> &g) {
        if (!g.has_value()) {
            return dp::nullopt;
        }

        using datapod::robot::geometry::box;
        using datapod::robot::geometry::cylinder;
        using datapod::robot::geometry::mesh;
        using datapod::robot::geometry::sphere;

        switch (g->type) {
        case Geometry::Type::Box:
            if (g->params.size() != 3) {
                return dp::nullopt;
            }
            return box(g->params[0], g->params[1], g->params[2]);
        case Geometry::Type::Cylinder:
            if (g->params.size() != 2) {
                return dp::nullopt;
            }
            return cylinder(g->params[0], g->params[1]);
        case Geometry::Type::Sphere:
            if (g->params.size() != 1) {
                return dp::nullopt;
            }
            return sphere(g->params[0]);
        case Geometry::Type::Mesh: {
            datapod::Array<dp::f64, 3> scale{g->mesh_scale[0], g->mesh_scale[1], g->mesh_scale[2]};
            return mesh(datapod::String(g->mesh_filename.c_str()), scale);
        }
        case Geometry::Type::Unknown:
        default:
            return dp::nullopt;
        }
    }

    static inline datapod::robot::Collision to_dp_collision(const Collision &c) {
        datapod::robot::Collision out;
        out.origin = c.origin;
        out.geom = to_dp_geometry(c.geometry).value_or(datapod::robot::geometry::box(0.0, 0.0, 0.0));
        return out;
    }

    static inline datapod::robot::Visual to_dp_visual(const Visual &v) {
        datapod::robot::Visual out;
        out.origin = v.origin;
        out.geom = to_dp_geometry(v.geometry).value_or(datapod::robot::geometry::box(0.0, 0.0, 0.0));
        return out;
    }

    static inline dp::Optional<datapod::robot::Inertial> to_dp_inertia(const dp::Optional<Inertial> &i) {
        if (!i.has_value()) {
            return dp::nullopt;
        }
        datapod::robot::Inertial out;
        out.origin = i->origin;
        out.mass = i->mass;
        out.ixx = i->ixx;
        out.ixy = i->ixy;
        out.ixz = i->ixz;
        out.iyy = i->iyy;
        out.iyz = i->iyz;
        out.izz = i->izz;
        return out;
    }

    static inline datapod::robot::Link to_dp_link(const Link &l) {
        datapod::robot::Link out;
        out.name = datapod::String(l.name.c_str());
        out.inertial = to_dp_inertia(l.inertial);
        for (const auto &v : l.visuals) {
            out.visuals.push_back(to_dp_visual(v));
        }
        for (const auto &c : l.collisions) {
            out.collisions.push_back(to_dp_collision(c));
        }
        return out;
    }

    static inline dp::Result<datapod::robot::Model>
    build_dp_model(const dp::String &root_link, const dp::Vector<Link> &links, const dp::Vector<Joint> &joints) {
        datapod::robot::Model out;

        dp::Vector<dp::String> link_names;
        link_names.reserve(links.size());
        for (const auto &l : links) {
            link_names.push_back(l.name);
        }

        auto find_link_id = [&](const dp::String &name) -> dp::Result<dp::u32> {
            for (dp::u32 i = 0; i < link_names.size(); ++i) {
                if (link_names[i] == name) {
                    return dp::Result<dp::u32>::ok(i);
                }
            }
            return dp::Result<dp::u32>::err(dp::Error::invalid_argument("unknown link name"));
        };

        for (const auto &l : links) {
            out.add_link(to_dp_link(l));
        }

        auto root_id_res = find_link_id(root_link);
        if (root_id_res.is_err()) {
            return dp::Result<datapod::robot::Model>::err(dp::Error::invalid_argument("root link not found"));
        }
        out.root = root_id_res.value();

        for (const auto &j : joints) {
            auto parent_id = find_link_id(j.parent_link);
            if (parent_id.is_err()) {
                return dp::Result<datapod::robot::Model>::err(dp::Error::invalid_argument("joint parent not found"));
            }
            auto child_id = find_link_id(j.child_link);
            if (child_id.is_err()) {
                return dp::Result<datapod::robot::Model>::err(dp::Error::invalid_argument("joint child not found"));
            }

            datapod::robot::Joint dj;
            dj.name = datapod::String(j.name.c_str());
            dj.type = to_dp_joint_type(j.type);
            dj.origin = j.parent_to_joint;
            dj.axis = datapod::Array<dp::f64, 3>{j.axis[0], j.axis[1], j.axis[2]};
            dj.limits = to_dp_joint_limits(j.limits);
            dj.dynamics = to_dp_joint_dynamics(j.dynamics);

            const dp::u32 joint_id = out.add_joint(std::move(dj));
            out.connect(parent_id.value(), child_id.value(), joint_id);
        }

        return dp::Result<datapod::robot::Model>::ok(out);
    }

    inline dp::Result<datapod::robot::Model> from_urdf_string(const dp::String &xml) {
        TiXmlDocument doc;
        doc.Parse(xml.c_str());
        if (doc.Error()) {
            return dp::Result<datapod::robot::Model>::err(dp::Error::invalid_argument("invalid xml"));
        }

        TiXmlElement *robot = doc.FirstChildElement("robot");
        if (!robot) {
            return dp::Result<datapod::robot::Model>::err(dp::Error::invalid_argument("missing <robot>"));
        }

        dp::String robot_name = dp_string(robot->Attribute("name"));
        if (robot_name.empty()) {
            return dp::Result<datapod::robot::Model>::err(dp::Error::invalid_argument("robot missing name"));
        }

        dp::Vector<Link> links;
        dp::Vector<Joint> joints;

        for (TiXmlElement *link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
            auto parsed = parse_link(link);
            if (parsed.is_err()) {
                return dp::Result<datapod::robot::Model>::err(parsed.error());
            }
            links.push_back(parsed.value());
        }
        if (links.empty()) {
            return dp::Result<datapod::robot::Model>::err(dp::Error::invalid_argument("no links"));
        }

        for (TiXmlElement *joint = robot->FirstChildElement("joint"); joint;
             joint = joint->NextSiblingElement("joint")) {
            auto parsed = parse_joint(joint);
            if (parsed.is_err()) {
                return dp::Result<datapod::robot::Model>::err(parsed.error());
            }
            joints.push_back(parsed.value());
        }

        auto root = find_root_link(links, joints);
        if (root.is_err()) {
            return dp::Result<datapod::robot::Model>::err(root.error());
        }

        auto dp_model = build_dp_model(root.value(), links, joints);
        if (dp_model.is_err()) {
            return dp::Result<datapod::robot::Model>::err(dp_model.error());
        }

        return dp::Result<datapod::robot::Model>::ok(dp_model.value());
    }

} // namespace robomod
