#!/usr/bin/env python3
"""운영 설정과 문서가 합의된 Localization 계약을 유지하는지 검사한다."""

import pathlib
import csv
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


def load_yaml(name):
    with (PACKAGE / "config" / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class ConfigurationContractTest(unittest.TestCase):
    def test_gnss_waits_cover_accepted_future_measurements(self):
        heading = load_yaml("imu_heading_calibration.yaml")["imu_heading_calibration"]
        gps = load_yaml("gps_reference.yaml")
        # At 1x replay/live, an accepted future stamp needs time for matching IMU/odom.
        self.assertGreaterEqual(heading["pending_wait_sec"],
                                heading["max_future_sec"] + heading["max_imu_gap_sec"])
        self.assertGreater(gps["timing"]["pending_wait_sec"],
                           gps["quality"]["max_future_stamp_sec"])
        self.assertLessEqual(heading["pending_wait_sec"], heading["history_sec"])

    def test_encoder_message_contract(self):
        erp42_package = (
            PACKAGE.parent / "erp42_msgs"
        )
        message_path = erp42_package / "msg" / "SerialFeedBack.msg"
        lines = [
            line.split("#", 1)[0].strip()
            for line in message_path.read_text(encoding="utf-8").splitlines()
        ]
        fields = [line for line in lines if line]
        self.assertEqual(
            [
                "uint8 GEAR_FORWARD=0",
                "uint8 GEAR_NEUTRAL=1",
                "uint8 GEAR_REVERSE=2",
                "uint8 MorA",
                "uint8 EStop",
                "uint8 Gear",
                "float64 speed",
                "float64 steer",
                "int16 brake",
                "int32 encoder",
                "uint8 alive",
            ],
            fields,
        )
        drive_path = erp42_package / "msg" / "DriveCmd.msg"
        drive_fields = [
            line.split("#", 1)[0].strip()
            for line in drive_path.read_text(encoding="utf-8").splitlines()
            if line.split("#", 1)[0].strip()
        ]
        self.assertEqual(
            [
                "uint8 GEAR_FORWARD=0",
                "uint8 GEAR_NEUTRAL=1",
                "uint8 GEAR_REVERSE=2",
                "uint16 KPH",
                "int16 Deg",
                "uint8 brake",
                "uint8 Gear",
                "uint8 EStop",
            ],
            drive_fields,
        )

    def test_encoder_rosserial_driver_is_wired_to_the_connected_uno(self):
        driver = load_yaml("encoder_driver.yaml")
        self.assertEqual(
            '/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_11254501101131313365-if00',
            driver["port"],
        )
        self.assertEqual(57600, driver["baud"])
        self.assertEqual("11254501101131313365", driver["hardware"]["usb_serial"])
        self.assertEqual("2341:0043", driver["hardware"]["usb_vid_pid"])
        for script in ("localization_command.sh", "localization_record_command.sh"):
            self.assertIn(
                '${MANDO_ENCODER_DEVICE:-' + driver["port"] + '}',
                (PACKAGE / "scripts" / script).read_text(encoding="utf-8"),
            )

        sensors = ET.parse(PACKAGE / "launch" / "sensors.launch").getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in sensors.findall("arg")
        }
        self.assertEqual("true", arguments["start_encoder_driver"])
        encoder_node = sensors.find(".//node[@pkg='rosserial_python']")
        self.assertIsNotNone(encoder_node)
        self.assertEqual("serial_node.py", encoder_node.attrib["type"])
        self.assertEqual(
            "$(arg encoder_driver_node_name)", encoder_node.attrib["name"]
        )
        self.assertEqual(
            "$(arg encoder_driver_config)",
            encoder_node.find("rosparam").attrib["file"],
        )

        bringup = (PACKAGE / "launch" / "bringup.launch").read_text(encoding="utf-8")
        self.assertIn('name="start_encoder_driver" default="true"', bringup)
        self.assertIn(
            'name="encoder_driver_config" value="$(arg encoder_driver_config)"',
            bringup,
        )
        dependencies = {
            element.text
            for element in ET.parse(PACKAGE / "package.xml").getroot().findall("exec_depend")
        }
        self.assertIn("rosserial_python", dependencies)

    def test_public_interfaces_and_units(self):
        config = load_yaml("localization_interfaces.yaml")
        self.assertEqual(1, config["schema_version"])
        self.assertEqual("/molit/localization/odometry", config["topics"]["output_odometry"])
        self.assertEqual("map", config["frames"]["map"])
        self.assertEqual("base_link", config["frames"]["base_link"])
        self.assertEqual("mando_encoder_serial", config["nodes"]["encoder_driver"])
        self.assertEqual("/erp42_serial/feedback", config["topics"]["encoder_state"])
        self.assertEqual("erp42_msgs/SerialFeedBack", config["message_types"]["encoder_state"])
        self.assertEqual("m/s", config["contracts"]["speed_unit"])
        self.assertEqual("rad/s", config["contracts"]["angular_velocity_unit"])
        public_topics = list(config["topics"].values())
        self.assertEqual(len(public_topics), len(set(public_topics)))
        internal_topics = list(config["internal_topics"].values())
        self.assertEqual(len(internal_topics), len(set(internal_topics)))
        self.assertFalse(set(public_topics) & set(internal_topics))
        self.assertEqual("localization_supervisor", config["nodes"]["supervisor"])
        self.assertEqual(
            "mando_localization/GpsGateReanchor",
            config["message_types"]["gps_gate_reanchor"],
        )
        self.assertNotEqual(
            config["services"]["local_ekf_set_pose"],
            config["services"]["global_ekf_set_pose"],
        )

    def test_external_nodes_use_fixed_internal_topics(self):
        adapter = (PACKAGE / "src" / "common" / "localization_interface_adapter.cpp").read_text(
            encoding="utf-8"
        )
        sensors = (PACKAGE / "launch" / "sensors.launch").read_text(encoding="utf-8")
        self.assertIn("/mando_localization/internal/driver/gps_navpvt", adapter)
        self.assertIn('from="~fix"', sensors)
        self.assertIn('from="~navpvt"', sensors)
        self.assertIn("/mando_localization/internal/driver/gps_navpvt", sensors)

    def test_calibrated_imu_is_the_common_fusion_input(self):
        interfaces = load_yaml("localization_interfaces.yaml")
        self.assertEqual(
            "/molit/localization/imu/calibrated",
            interfaces["topics"]["imu_calibrated"],
        )
        self.assertEqual("sensor_msgs/Imu", interfaces["message_types"]["imu_calibrated"])
        self.assertEqual(
            "diagnostic_msgs/DiagnosticArray",
            interfaces["message_types"]["imu_calibration_status"],
        )
        imu_topics = [interfaces["topics"][key] for key in
                      ("imu_data", "imu_normalized", "imu_calibrated")]
        self.assertEqual(3, len(set(imu_topics)))
        for config in ("ekf_local.yaml", "ekf_global.yaml"):
            self.assertEqual("/mando_localization/internal/ekf/imu", load_yaml(config)["imu0"])

        local = ET.parse(PACKAGE / "launch" / "local_fusion.launch").getroot()
        group = local.find("group[@if='$(arg start_calibrated_imu)']")
        self.assertIsNotNone(group, "External normalized IMU must still pass through calibration")
        node = group.find("node[@type='calibrated_imu_node.py']")
        self.assertIsNotNone(node)
        files = [item.attrib["file"] for item in node.findall("rosparam")]
        self.assertIn("$(arg interfaces_config)", files)
        self.assertIn("$(arg imu_heading_config)", files)
        bringup = ET.parse(PACKAGE / "launch" / "bringup.launch").getroot()
        for launch in (local, bringup):
            self.assertEqual(
                "true", launch.find("arg[@name='start_calibrated_imu']").attrib["default"]
            )
        include = bringup.find("include[@file='$(find mando_localization)/launch/local_fusion.launch']")
        for name in ("start_calibrated_imu", "imu_heading_config", "calibrated_imu_node_name"):
            self.assertEqual("$(arg " + name + ")", include.find("arg[@name='" + name + "']").attrib["value"])

    def test_tf_has_single_dynamic_owners_and_lidar_static_transform_enabled(self):
        config = load_yaml("tf_configuration.yaml")
        reference = config["base_link_reference"]
        self.assertEqual("rear_axle_center", reference["origin"])
        self.assertEqual("forward", reference["x_axis"])
        self.assertEqual("left", reference["y_axis"])
        self.assertEqual("up", reference["z_axis"])
        dynamic = {
            (item["parent_frame"], item["child_frame"]): item["owner_node"]
            for item in config["dynamic_transforms"]
        }
        self.assertEqual("odometry_gps_global_ekf", dynamic[("map", "odom")])
        self.assertEqual("imu_encoder_local_ekf", dynamic[("odom", "base_link")])
        static = {item["child_frame"]: item for item in config["static_transforms"]}
        children = set()
        for transform in static.values():
            self.assertNotIn(transform["child_frame"], children)
            children.add(transform["child_frame"])
        for child_frame in ("gps_link",):
            self.assertFalse(static[child_frame]["enabled"])
            self.assertEqual("unmeasured", static[child_frame]["calibration_state"])
        self.assertTrue(static["imu_link"]["enabled"])
        self.assertEqual("verified", static["imu_link"]["calibration_state"])
        self.assertEqual([0.0, 0.0, 0.0], static["imu_link"]["rotation_rpy_deg"])
        self.assertTrue(static["laser_link"]["enabled"])
        self.assertEqual("measured", static["laser_link"]["calibration_state"])
        self.assertEqual("2026-09-05", static["laser_link"]["measured_at"])
        self.assertTrue(static["laser_link"]["source"])
        self.assertEqual([0.20, 0.0, 0.0], static["imu_link"]["translation_m"])
        self.assertEqual([0.65, 0.0, 0.0], static["gps_link"]["translation_m"])
        self.assertEqual([1.05, 0.0, 0.0], static["laser_link"]["translation_m"])
        self.assertEqual([180.0, 0.0, 180.0], static["laser_link"]["rotation_rpy_deg"])

    def test_filters_obey_tf_contract(self):
        local = load_yaml("ekf_local.yaml")
        global_filter = load_yaml("ekf_global.yaml")
        self.assertEqual("odom", local["world_frame"])
        self.assertTrue(local["publish_tf"])
        self.assertTrue(local["predict_to_current_time"])
        self.assertEqual(0.0, local["transform_timeout"])
        self.assertEqual(0.0, global_filter["transform_timeout"])
        self.assertEqual("map", global_filter["world_frame"])
        self.assertTrue(global_filter["publish_tf"])
        self.assertTrue(local["twist0_config"][7])
        self.assertTrue(global_filter["twist0_config"][7])
        self.assertNotIn("odom0", global_filter)

        encoder = load_yaml("encoder_calibration.yaml")["encoder"]
        lateral = encoder["lateral_velocity_constraint"]
        self.assertNotIn("enabled", lateral)
        self.assertEqual(0.01, lateral["variance_m2ps2"])
        self.assertEqual("model_assumption", lateral["calibration_state"])
        self.assertTrue(lateral["source"])

    def test_measured_imu_identity_and_fail_closed_covariance(self):
        imu = load_yaml("imu_driver.yaml")
        self.assertEqual("/dev/imu", imu["port"])
        self.assertEqual("0388BD48", imu["device_id"])
        self.assertEqual(115200, imu["baudrate"])
        self.assertTrue(imu["covariance_override"]["enabled"])
        self.assertEqual("measured", imu["covariance_override"]["calibration_state"])
        self.assertTrue(imu["covariance_override"]["measured_at"])
        self.assertIn("imu_stationary_20260831", imu["covariance_override"]["source"])
        self.assertTrue(imu["imu"]["require_positive_covariance_diagonal"])

    def test_gps_device_diagnostic_matches_driver(self):
        self.assertEqual(load_yaml("gps_driver.yaml")["device"],
                         load_yaml("status_policy.yaml")["devices"]["gps"])

    def test_gps_driver_forces_volatile_ubx_navpvt(self):
        gps = load_yaml("gps_driver.yaml")
        self.assertFalse(gps["use_ros_time"])
        self.assertTrue(gps["require_valid_utc"])
        self.assertEqual(
            "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00",
            gps["device"],
        )
        self.assertEqual(460800, gps["uart1"]["baudrate"])
        self.assertEqual(1, gps["uart1"]["in"])
        self.assertEqual(1, gps["uart1"]["out"])
        self.assertTrue(gps["config_on_startup"])
        self.assertFalse(gps["save_on_shutdown"])
        self.assertEqual("auto", gps["fix_mode"])
        self.assertEqual(0, gps["tmode3"])
        self.assertTrue(gps["publish"]["nav"]["pvt"])
        self.assertFalse(gps["publish"]["nav"]["relposned"])

    def test_gps_timing_history_and_clock_gate_are_wired(self):
        gps = load_yaml("gps_reference.yaml")
        global_filter = load_yaml("ekf_global.yaml")
        self.assertTrue(global_filter["smooth_lagged_data"])
        self.assertTrue(global_filter["predict_to_current_time"])
        self.assertFalse(global_filter["permit_corrected_publication"])
        self.assertGreater(global_filter["history_length"],
                           gps["quality"]["max_message_age_sec"])
        sensors = ET.parse(PACKAGE / "launch" / "sensors.launch").getroot()
        driver = sensors.find(".//node[@pkg='ublox_gps']")
        self.assertIn("check_time_sync.py", driver.attrib["launch-prefix"])
        self.assertIn("--exec", driver.attrib["launch-prefix"])
        gps_launch = ET.parse(PACKAGE / "launch" / "gps_fusion.launch").getroot()
        self.assertIsNotNone(gps_launch.find(".//node[@type='sensor_timing_monitor.py']"))
        interfaces = load_yaml("localization_interfaces.yaml")
        self.assertEqual("/mando_localization/internal/timing/clock_ready",
                         interfaces["internal_topics"]["clock_ready"])

    def test_operational_datum_is_not_faked(self):
        gps_reference = load_yaml("gps_reference.yaml")
        reference = gps_reference["reference"]
        self.assertEqual("first_fix", reference["mode"])
        self.assertFalse(reference["measured"])
        self.assertEqual("", reference["measured_at"])
        self.assertEqual("", reference["source"])
        self.assertEqual(
            2.0,
            gps_reference["quality"]["max_reanchor_candidate_distance_m"],
        )
        self.assertEqual(30.0, gps_reference["quality"]["max_step_distance_m"])
        self.assertEqual(30.0,
                         gps_reference["quality"]["max_position_innovation_m"])
        lever_arm = gps_reference["lever_arm"]
        self.assertNotIn("enabled", lever_arm)
        self.assertEqual(0.65, lever_arm["x_m"])
        self.assertEqual(0.0, lever_arm["y_m"])
        self.assertEqual(0.10, lever_arm["max_yaw_stamp_skew_sec"])
        self.assertEqual("provisional", lever_arm["calibration_state"])
        self.assertTrue(lever_arm["source"])

    def test_dead_reckoning_is_bounded(self):
        policy = load_yaml("status_policy.yaml")
        self.assertEqual(
            30.0,
            policy["absolute_sources"]["max_global_consistency_distance_m"],
        )
        self.assertEqual(200.0, policy["dead_reckoning"]["max_duration_sec"])
        self.assertEqual(1000.0, policy["dead_reckoning"]["max_distance_m"])
        self.assertEqual("first_exceeded", policy["dead_reckoning"]["limit_policy"])
        self.assertFalse(policy["output_gate"]["publish_last_pose_when_invalid"])

    def test_long_outage_recovery_is_bounded_and_gps_only_is_disabled(self):
        policy = load_yaml("relocalization_policy.yaml")["relocalization"]
        self.assertEqual(2.0, policy["long_outage_sec"])
        self.assertEqual(1.0, policy["reanchor_ack_timeout_sec"])
        self.assertFalse(policy["gps_only"]["automatic_reset_enabled"])
        self.assertTrue(policy["gps_only"]["require_measured_datum"])
        self.assertEqual(5, policy["gps_only"]["required_consecutive_candidates"])
        self.assertEqual(3, policy["gps_only"]["required_global_confirmations"])

    def test_supervisor_owns_public_status_and_ekf_reset_services_are_split(self):
        safety = (PACKAGE / "launch" / "safety_and_tf.launch").read_text(
            encoding="utf-8"
        )
        local = (PACKAGE / "launch" / "local_fusion.launch").read_text(encoding="utf-8")
        global_fusion = (PACKAGE / "launch" / "global_fusion.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('type="localization_supervisor_node"', safety)
        self.assertIn('name="start_supervisor"', safety)
        self.assertIn('if="$(arg start_supervisor)"', safety)
        self.assertIn("relocalization_policy_config", safety)
        self.assertIn("output_gate/allow_unbounded_position_variance", safety)
        self.assertIn("not arg('enable_gps_fusion')", safety)
        self.assertNotIn('type="localization_status_manager_node"', safety)
        self.assertIn("/mando_localization/internal/ekf/local_set_pose", local)
        self.assertIn("/mando_localization/internal/ekf/global_set_pose", global_fusion)
        self.assertTrue((PACKAGE / "msg" / "GpsGateReanchor.msg").is_file())

    def test_only_gps_absolute_pose_is_fused(self):
        config = load_yaml("ekf_global.yaml")
        self.assertEqual("/mando_localization/internal/ekf/gps_pose", config["pose0"])
        self.assertNotIn("pose1", config)
        self.assertEqual({"gps_enabled"}, {
            key for key in load_yaml("status_policy.yaml")["absolute_sources"]
            if key.endswith("_enabled")
        })
        for name in ("bringup.launch", "safety_and_tf.launch"):
            launch = ET.parse(PACKAGE / "launch" / name).getroot()
            self.assertNotIn("enable_lidar_localization", {
                item.attrib["name"] for item in launch.findall("arg")
            })

    def test_single_command_runs_collection_stack_without_recording(self):
        command = (PACKAGE / "scripts" / "localization_command.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "roslaunch mando_localization map_data_collection.launch", command
        )
        self.assertIn("start_lidar_driver", command)
        self.assertIn('enable_gps_fusion:="${GPS_ARG}"', command)
        self.assertIn("start_rviz:=true", command)
        self.assertIn("start_recording:=false", command)
        self.assertGreater(
            command.rfind("start_recording:=false"), command.find('"$@"')
        )
        self.assertNotIn("enable_lidar_localization:=true", command)
        self.assertNotIn("localization_live_sensor_debug", command)
        self.assertNotIn("HL-FMA2026-stier", command)

    def test_map_data_collection_records_source_topics_without_amcl(self):
        config = load_yaml("lidar_driver.yaml")
        self.assertEqual("/dev/lidar", config["serial_port"])
        self.assertEqual(1000000, config["serial_baudrate"])
        self.assertEqual("laser_link", config["frame_id"])
        self.assertFalse(config["inverted"])
        self.assertTrue(config["angle_compensate"])
        self.assertEqual(10.0, config["scan_frequency"])

        collection = ET.parse(
            PACKAGE / "launch" / "map_data_collection.launch"
        ).getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in collection.findall("arg")
        }
        self.assertEqual("true", arguments["start_lidar_driver"])
        self.assertEqual("$(arg start_gps_driver)", arguments["enable_gps_fusion"])
        self.assertEqual("true", arguments["start_recording"])
        self.assertEqual("/dev/lidar", arguments["lidar_serial_port"])
        self.assertEqual("laser_link", arguments["lidar_frame_id"])
        self.assertEqual(
            "^(/molit/localization/(path|markers)|"
            "/mando_localization/visualization/debug/scene)$",
            arguments["bag_exclude_regex"],
        )
        self.assertEqual(
            "/molit/sensors/lidar/scan", arguments["lidar_scan_topic"]
        )

        lidar_node = collection.find(".//node[@pkg='rplidar_ros']")
        self.assertIsNotNone(lidar_node)
        self.assertEqual("rplidarNode", lidar_node.attrib["type"])
        self.assertEqual("true", lidar_node.attrib["required"])
        scan_remap = lidar_node.find("remap[@from='scan']")
        self.assertEqual("$(arg lidar_scan_topic)", scan_remap.attrib["to"])

        bringup = collection.find("include")
        included_args = {
            item.attrib["name"]: item.attrib["value"]
            for item in bringup.findall("arg")
        }
        self.assertEqual("true", included_args["start_static_tf_publisher"])
        self.assertEqual("$(arg enable_gps_fusion)", included_args["enable_gps_fusion"])
        self.assertEqual("$(arg rviz_config)", included_args["rviz_config"])

        visualization = ET.parse(PACKAGE / "launch" / "visualization.launch").getroot()
        self.assertIsNone(visualization.find(".//node[@pkg='rviz']"))
        self.assertIsNotNone(visualization.find(".//include[@file='$(find mando_localization)/launch/viewer.launch']"))
        self.assertEqual("$(find mando_localization)/config/localization_viewer.yaml", arguments["rviz_config"])

        recorder = collection.find(".//node[@pkg='rosbag']")
        self.assertIsNotNone(recorder)
        self.assertEqual("record", recorder.attrib["type"])
        self.assertEqual("true", recorder.attrib["required"])
        for token in (
            "-a",
            "--exclude='$(arg bag_exclude_regex)'",
            "--lz4",
            "--split",
            "--repeat-latched",
            "--min-space=$(arg bag_min_space)",
        ):
            self.assertIn(token, recorder.attrib["args"])

        command = (
            PACKAGE / "scripts" / "localization_record_command.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("map_data_collection.launch", command)
        self.assertIn("용인운전면허장 로스백", command)
        self.assertIn("--allow-missing-sensors", command)
        self.assertIn('bag_exclude_regex:="${BAG_EXCLUDE_REGEX}"', command)

        dependencies = {
            element.text
            for element in ET.parse(PACKAGE / "package.xml")
            .getroot()
            .findall("exec_depend")
        }
        self.assertIn("rosbag", dependencies)
        self.assertIn("rplidar_ros", dependencies)

    def test_viewer_and_front_scan_use_current_interfaces(self):
        interfaces = load_yaml("localization_interfaces.yaml")
        viewer = load_yaml("localization_viewer.yaml")
        for key, interface in (
            ("local", "local_odometry"), ("global", "global_odometry"),
            ("gps", "gps_fix"), ("speed", "encoder_state"),
            ("state", "state"), ("valid", "valid"),
            ("imu_raw", "imu_data"), ("imu_normalized", "imu_normalized"),
            ("imu_calibrated", "imu_calibrated"),
        ):
            with self.subTest(topic=key):
                self.assertEqual(interfaces["topics"][interface], viewer["topics"][key])
        self.assertEqual(interfaces["internal_topics"]["imu_calibration_status"],
                         viewer["topics"]["calibration"])
        front = load_yaml("lidar_front_visualization.yaml")
        self.assertEqual(front["output_topic"], viewer["topics"]["scan"])
        self.assertNotEqual(interfaces["topics"]["lidar_scan"], front["output_topic"])
        self.assertEqual(-90.0, front["sector"]["min_angle_deg"])
        self.assertEqual(90.0, front["sector"]["max_angle_deg"])
        visualization = ET.parse(PACKAGE / "launch" / "visualization.launch").getroot()
        self.assertIsNotNone(visualization.find(
            ".//node[@type='lidar_front_scan_visualizer.py']"
        ))
        for filename, arg in (("bringup.launch", "rviz_config"),
                              ("visualization.launch", "rviz_config"),
                              ("map_data_collection.launch", "rviz_config"),
                              ("viewer.launch", "config")):
            launch = ET.parse(PACKAGE / "launch" / filename).getroot()
            self.assertEqual("$(find mando_localization)/config/localization_viewer.yaml",
                             launch.find("arg[@name='%s']" % arg).attrib["default"])
        viewer_launch = ET.parse(PACKAGE / "launch" / "viewer.launch").getroot()
        self.assertEqual("localization_viewer.py", viewer_launch.find("node").attrib["type"])
        self.assertEqual("live", viewer_launch.find("arg[@name='mode']").attrib["default"])
        dependencies = {e.text for e in ET.parse(PACKAGE / "package.xml")
                        .getroot().findall("exec_depend")}
        self.assertTrue({"rospy", "rviz", "python_qt_binding"}.issubset(dependencies))

    def test_launch_files_are_valid_xml(self):
        launch_files = list((PACKAGE / "launch").glob("*.launch"))
        self.assertGreaterEqual(len(launch_files), 5)
        for launch_file in launch_files:
            with self.subTest(launch=launch_file.name):
                self.assertEqual("launch", ET.parse(launch_file).getroot().tag)

    def test_initial_heading_matches_rddf_and_both_filter_states(self):
        config = load_yaml("initial_heading.yaml")
        initial = config["initial_heading"]
        source, index = initial["source"].split("#index=")
        with (PACKAGE / source).open(encoding="utf-8-sig", newline="") as stream:
            row = list(csv.DictReader(stream))[int(index)]
        self.assertAlmostEqual(float(row["path_yaw_rad"]), initial["yaw_rad"])
        self.assertEqual(15, len(config["initial_state"]))
        self.assertEqual(initial["yaw_rad"], config["initial_state"][5])
        self.assertTrue(all(v == 0 for i, v in enumerate(config["initial_state"]) if i != 5))
        for filename, nodes in (("local_fusion.launch", ("calibrated_imu_node.py", "ekf_localization_node")),
                                ("global_fusion.launch", ("ekf_localization_node",))):
            launch = ET.parse(PACKAGE / "launch" / filename).getroot()
            for node_type in nodes:
                node = launch.find(".//node[@type='%s']" % node_type)
                load = node.find("rosparam[@file='$(arg initial_heading_config)']")
                self.assertIsNotNone(load)
                self.assertEqual("$(arg initialize_heading)", load.attrib["if"])

    def test_navsat_transform_is_not_a_second_gps_path(self):
        self.assertFalse((PACKAGE / "config" / "navsat_transform.yaml").exists())
        launch_and_config = "\n".join(
            path.read_text(encoding="utf-8")
            for folder in ("launch", "config")
            for path in (PACKAGE / folder).glob("*")
            if path.is_file()
        )
        self.assertNotIn("navsat_transform", launch_and_config)

    def test_required_document_set_exists(self):
        for relative_path in (
            "docs/README.md",
            "docs/architecture.md",
            "docs/configuration.md",
            "docs/tf_frames.md",
            "docs/gps_quality_and_recovery.md",
            "docs/status_and_recovery.md",
        ):
            with self.subTest(path=relative_path):
                self.assertGreater((PACKAGE / relative_path).stat().st_size, 200)
        gps_document = (PACKAGE / "docs" / "gps_quality_and_recovery.md").read_text(
            encoding="utf-8"
        )
        for token in (
            "STATUS_NO_FIX",
            "STATUS_SBAS_FIX",
            "STATUS_GBAS_FIX",
            "COVARIANCE_TYPE_UNKNOWN",
            "30.0 m",
            "GpsGateReanchor",
            "transaction_id",
            "automatic_reset_enabled: false",
        ):
            self.assertIn(token, gps_document)


if __name__ == "__main__":
    unittest.main()
