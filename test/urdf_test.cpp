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

    auto parsed = agent47::from_urdf_string(xml);
    REQUIRE(parsed.is_ok());

    const auto &m = parsed.value();

    CHECK(m.links.size() == 2);
    CHECK(m.joints.size() == 1);
    CHECK(m.root != datapod::robot::kInvalidId);
    CHECK(m.links[m.root].name == datapod::String("base_link"));

    // Props: robot-level
    CHECK(m.props.find(datapod::String("flatsim.turning.radius")) != m.props.end());
    CHECK(m.props.at(datapod::String("flatsim.turning.radius")) == datapod::String("1.23"));

    // Props: link-level
    CHECK(m.links[m.root].props.find(datapod::String("flatsim.link_tag")) != m.links[m.root].props.end());
    CHECK(m.links[m.root].props.at(datapod::String("flatsim.link_tag")) == datapod::String("base"));

    // Props: joint-level
    CHECK(m.joints[0].props.find(datapod::String("flatsim.side")) != m.joints[0].props.end());
    CHECK(m.joints[0].props.at(datapod::String("flatsim.side")) == datapod::String("left"));
    CHECK(m.joints[0].props.find(datapod::String("flatsim.throttle_max")) != m.joints[0].props.end());
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

    auto parsed = agent47::from_urdf_string(xml);
    REQUIRE(parsed.is_ok());

    const auto &m = parsed.value();
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
