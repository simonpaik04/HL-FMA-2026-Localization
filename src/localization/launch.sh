#!/usr/bin/env bash
set -eo pipefail

# The workspace bringup starts localization; sensor/vehicle drivers run separately.
# Estimator, quality gates, TF calibration and fusion defaults remain unchanged.
exec roslaunch mando_localization bringup.launch \
  start_encoder_driver:=false \
  start_imu_driver:=false \
  start_gps_driver:=false \
  start_rviz:=false \
  "$@"
