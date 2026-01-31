#include <doctest/doctest.h>

#include <agent47/model/urdf.hpp>

TEST_CASE("urdf: parses basic model and captures extensions") {
    const dp::String xml = R"_URDF(
<?xml version="1.0"?>
<robot name="mini">
  <flatsim>
    <turning radius="1.23"/>
  </flatsim>

  <link name="base_link">
    <flatsim link_tag="base"/>
    <visual>
      <geometry>
        <box size="1 2 3"/>
      </geometry>
    </visual>
  </link>

  <link name="wheel_link"/>

  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <flatsim side="left" throttle_max="0.2"/>
  </joint>
</robot>
)_URDF";

    auto parsed = robomod::from_urdf_string(xml);
    REQUIRE(parsed.is_ok());

    const auto &u = parsed.value();

    CHECK(u.model.links.size() == 2);
    CHECK(u.model.joints.size() == 1);
    CHECK(u.model.root != datapod::robot::kInvalidId);
    CHECK(u.model.links[u.model.root].name == datapod::String("base_link"));

    // Extensions captured at the right scope
    CHECK(u.ext.robot.size() == 1);
    CHECK(u.ext.links.find(dp::String("base_link")) != u.ext.links.end());
    CHECK(u.ext.joints.find(dp::String("wheel_joint")) != u.ext.joints.end());

    // Spot-check content contains tag name
    CHECK(u.ext.robot[0].find("flatsim") != dp::String::npos);
    CHECK(u.ext.links.at(dp::String("base_link"))[0].find("flatsim") != dp::String::npos);
    CHECK(u.ext.joints.at(dp::String("wheel_joint"))[0].find("throttle_max") != dp::String::npos);
}

TEST_CASE("urdf: parses joint limits and dynamics") {
    const dp::String xml = R"_URDF(
<robot name="mini">
  <link name="a"/>
  <link name="b"/>
  <joint name="j" type="revolute">
    <parent link="a"/>
    <child link="b"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="2" effort="3" velocity="4"/>
    <dynamics damping="0.1" friction="0.2"/>
  </joint>
</robot>
)_URDF";

    auto parsed = robomod::from_urdf_string(xml);
    REQUIRE(parsed.is_ok());

    const auto &m = parsed.value().model;
    REQUIRE(m.joints.size() == 1);

    CHECK(m.joints[0].limits.has_value());
    CHECK(m.joints[0].limits->lower == doctest::Approx(-1.0));
    CHECK(m.joints[0].limits->upper == doctest::Approx(2.0));
    CHECK(m.joints[0].limits->effort == doctest::Approx(3.0));
    CHECK(m.joints[0].limits->velocity == doctest::Approx(4.0));

    CHECK(m.joints[0].dynamics.has_value());
    CHECK(m.joints[0].dynamics->damping == doctest::Approx(0.1));
    CHECK(m.joints[0].dynamics->friction == doctest::Approx(0.2));
}
