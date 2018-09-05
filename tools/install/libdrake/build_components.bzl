# -*- python -*-

# Should include everything any consumer of Drake would ever need.
#
# Do not update this list by hand; instead, run build_components_refresh.py.
LIBDRAKE_COMPONENTS = [
    "//automotive",
    "//automotive/maliput/api",
    "//automotive/maliput/dragway",
    "//automotive/maliput/monolane",
    "//automotive/maliput/multilane",
    "//automotive/maliput/rndf",
    "//automotive/maliput/simplerulebook",
    "//automotive/maliput/utility",
    "//bindings/pydrake/systems:systems_pybind",  # unpackaged
    "//bindings/pydrake/util:cpp_param_pybind",  # unpackaged
    "//bindings/pydrake/util:cpp_template_pybind",  # unpackaged
    "//bindings/pydrake/util:deprecation_pybind",  # unpackaged
    "//bindings/pydrake/util:drake_optional_pybind",  # unpackaged
    "//bindings/pydrake/util:eigen_geometry_pybind",  # unpackaged
    "//bindings/pydrake/util:eigen_pybind",  # unpackaged
    "//bindings/pydrake/util:type_pack",  # unpackaged
    "//bindings/pydrake/util:type_safe_index_pybind",  # unpackaged
    "//bindings/pydrake/util:wrap_function",  # unpackaged
    "//bindings/pydrake/util:wrap_pybind",  # unpackaged
    "//bindings/pydrake:autodiff_types_pybind",  # unpackaged
    "//bindings/pydrake:pydrake_pybind",  # unpackaged
    "//bindings/pydrake:symbolic_types_pybind",  # unpackaged
    "//common",
    "//common/proto",
    "//common/trajectories",
    "//common:drake_marker_shared_library",  # unpackaged
    "//common:text_logging_gflags_h",  # unpackaged
    "//geometry",
    "//geometry/query_results",
    "//lcm",
    "//manipulation/perception",
    "//manipulation/planner",
    "//manipulation/scene_generation:random_clutter_generator",  # unpackaged
    "//manipulation/scene_generation:simulate_plant_to_rest",  # unpackaged
    "//manipulation/schunk_wsg",
    "//manipulation/sensors",
    "//manipulation/util",
    "//math",
    "//multibody/benchmarks/acrobot",
    "//multibody/benchmarks/free_body",
    "//multibody/benchmarks/inclined_plane",
    "//multibody/benchmarks/kuka_iiwa_robot",
    "//multibody/benchmarks/mass_damper_spring",
    "//multibody/benchmarks/pendulum",
    "//multibody/collision",
    "//multibody/constraint",
    "//multibody/joints",
    "//multibody/multibody_tree",
    "//multibody/multibody_tree/implicit_stribeck:implicit_stribeck_solver",  # unpackaged  # noqa
    "//multibody/multibody_tree/math",
    "//multibody/multibody_tree/multibody_plant",
    "//multibody/multibody_tree/multibody_plant:contact_results_to_lcm",  # unpackaged  # noqa
    "//multibody/multibody_tree/parsing",
    "//multibody/parsers",
    "//multibody/parsing",
    "//multibody/rigid_body_plant:compliant_contact_model",  # unpackaged
    "//multibody/rigid_body_plant:compliant_material",  # unpackaged
    "//multibody/rigid_body_plant:contact_results",  # unpackaged
    "//multibody/rigid_body_plant:contact_results_to_lcm",  # unpackaged
    "//multibody/rigid_body_plant:create_load_robot_message",  # unpackaged
    "//multibody/rigid_body_plant:drake_visualizer",  # unpackaged
    "//multibody/rigid_body_plant:frame_visualizer",  # unpackaged
    "//multibody/rigid_body_plant:rigid_body_plant",  # unpackaged
    "//multibody/rigid_body_plant:rigid_body_plant_bridge",  # unpackaged
    "//multibody/shapes",
    "//multibody:approximate_ik",  # unpackaged
    "//multibody:global_inverse_kinematics",  # unpackaged
    "//multibody:inverse_kinematics",  # unpackaged
    "//multibody:kinematics_cache",  # unpackaged
    "//multibody:kinematics_cache_helper",  # unpackaged
    "//multibody:resolve_center_of_pressure",  # unpackaged
    "//multibody:rigid_body",  # unpackaged
    "//multibody:rigid_body_actuator",  # unpackaged
    "//multibody:rigid_body_constraint",  # unpackaged
    "//multibody:rigid_body_distance_constraint",  # unpackaged
    "//multibody:rigid_body_frame",  # unpackaged
    "//multibody:rigid_body_loop",  # unpackaged
    "//multibody:rigid_body_tree",  # unpackaged
    "//multibody:rigid_body_tree_alias_groups",  # unpackaged
    "//multibody:rigid_body_tree_alias_groups_proto",  # unpackaged
    "//multibody:rigid_body_tree_construction",  # unpackaged
    "//perception",
    "//solvers",
    "//systems/analysis",
    "//systems/controllers",
    "//systems/controllers/plan_eval",
    "//systems/controllers/qp_inverse_dynamics",
    "//systems/estimators",
    "//systems/framework",
    "//systems/lcm",
    "//systems/learning",
    "//systems/plants/spring_mass_system",
    "//systems/primitives",
    "//systems/rendering",
    "//systems/robotInterfaces",
    "//systems/sensors",
    "//systems/trajectory_optimization",
    "//util",
]
