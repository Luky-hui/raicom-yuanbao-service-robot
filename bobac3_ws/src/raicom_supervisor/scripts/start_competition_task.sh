#!/usr/bin/env bash
set -euo pipefail

TASK="${1:-}"
TARGET_OBJECT="${2:-}"

if [[ -z "${TASK}" ]]; then
  echo "Usage: start_competition_task.sh provincial_welcome|provincial_patrol|provincial_patrol_scripted"
  exit 2
fi

set +u
source /opt/ros/noetic/setup.bash
source /home/yuanbao/bobac3_ws/devel/setup.bash
source /home/yuanbao/ros_workspace/devel/setup.bash --extend
set -u

setup_audio() {
  local preferred_source="${RAICOM_AUDIO_SOURCE:-alsa_input.usb-Generalplus_Usb_Audio_Device-00.mono-fallback}"
  local preferred_sink="${RAICOM_AUDIO_SINK:-alsa_output.usb-USB_AUDIO_DAC_USB_AUDIO_DAC-00.analog-stereo}"
  local source_volume="${RAICOM_AUDIO_SOURCE_VOLUME:-33%}"
  local sink_volume="${RAICOM_AUDIO_SINK_VOLUME:-100%}"
  local mic_card="${RAICOM_MIC_CARD:-2}"
  local mic_gain="${RAICOM_MIC_GAIN:-10}"
  local selected_source=""
  local selected_sink=""

  if pactl list short sources >/tmp/raicom_sources.$$ 2>/tmp/raicom_sources_err.$$; then
    if awk '{print $2}' /tmp/raicom_sources.$$ | grep -qx "${preferred_source}"; then
      selected_source="${preferred_source}"
    else
      selected_source="$(pactl get-default-source 2>/dev/null || true)"
      if [[ "${selected_source}" == *.monitor ]] || \
         ! awk '{print $2}' /tmp/raicom_sources.$$ | grep -qx "${selected_source}"; then
        selected_source="$(awk 'index($2, ".monitor")==0 {print $2; exit}' /tmp/raicom_sources.$$)"
      fi
      echo "WARN audio source ${preferred_source} not found; using ${selected_source:-none}" >&2
    fi
    if [[ -n "${selected_source}" ]]; then
      pactl set-default-source "${selected_source}" || true
      pactl set-source-volume "${selected_source}" "${source_volume}" || true
      export PULSE_SOURCE="${selected_source}"
    fi
  else
    echo "WARN pactl source list failed; audio input was not configured." >&2
    cat /tmp/raicom_sources_err.$$ >&2 || true
  fi
  rm -f /tmp/raicom_sources.$$ /tmp/raicom_sources_err.$$

  if pactl list short sinks >/tmp/raicom_sinks.$$ 2>/tmp/raicom_sinks_err.$$; then
    if awk '{print $2}' /tmp/raicom_sinks.$$ | grep -qx "${preferred_sink}"; then
      selected_sink="${preferred_sink}"
    else
      selected_sink="$(pactl get-default-sink 2>/dev/null || true)"
      if ! awk '{print $2}' /tmp/raicom_sinks.$$ | grep -qx "${selected_sink}"; then
        selected_sink="$(awk '{print $2; exit}' /tmp/raicom_sinks.$$)"
      fi
      echo "WARN audio sink ${preferred_sink} not found; using ${selected_sink:-none}" >&2
    fi
    if [[ -n "${selected_sink}" ]]; then
      pactl set-default-sink "${selected_sink}" || true
      pactl set-sink-volume "${selected_sink}" "${sink_volume}" || true
    fi
  else
    echo "WARN pactl sink list failed; audio output was not configured." >&2
    cat /tmp/raicom_sinks_err.$$ >&2 || true
  fi
  rm -f /tmp/raicom_sinks.$$ /tmp/raicom_sinks_err.$$

  amixer -c "${mic_card}" sset Mic "${mic_gain}" cap >/dev/null 2>&1 || \
    echo "WARN amixer mic setup failed for card ${mic_card}; continuing." >&2
}

setup_audio


require_navigation_ready() {
  echo 'Checking navigation stack readiness...'
  if ! timeout 3 rostopic list >/tmp/raicom_topics.$$ 2>/tmp/raicom_topics_err.$$; then
    echo 'ERROR: ROS master is not reachable. Please start navigation first:' >&2
    echo '  roslaunch bobac3_navigation demo_nav_2d.launch open_nav_rviz:=true gazebo_gui:=true map_file_name:=reicom' >&2
    rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
    exit 3
  fi

  if ! grep -qx '/map' /tmp/raicom_topics.$$; then
    echo 'ERROR: /map is not available. Start navigation first and wait for map_server.' >&2
    echo 'Command:' >&2
    echo '  roslaunch bobac3_navigation demo_nav_2d.launch open_nav_rviz:=true gazebo_gui:=true map_file_name:=reicom' >&2
    rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
    exit 3
  fi

  if ! grep -qx '/move_base/status' /tmp/raicom_topics.$$; then
    echo 'ERROR: /move_base/status is not available. move_base is not ready.' >&2
    rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
    exit 3
  fi
  rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$

  if ! timeout 5 rostopic echo -n 1 /map/header >/dev/null 2>&1; then
    echo 'ERROR: /map exists but no map message was received within 5 seconds.' >&2
    exit 3
  fi

  if ! timeout 8 python3 - <<'PY_TF_CHECK' >/tmp/raicom_tf.$$ 2>&1
import rospy
import tf
rospy.init_node('raicom_nav_ready_tf_check', anonymous=True, disable_signals=True)
listener = tf.TransformListener()
listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(6.0))
translation, rotation = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
print('TF_OK map base_footprint %.3f %.3f %.3f' % (translation[0], translation[1], translation[2]))
PY_TF_CHECK
  then
    echo 'ERROR: TF map -> base_footprint is not ready. Wait until AMCL/localization is ready.' >&2
    cat /tmp/raicom_tf.$$ >&2 || true
    rm -f /tmp/raicom_tf.$$
    exit 3
  fi
  cat /tmp/raicom_tf.$$ || true
  rm -f /tmp/raicom_tf.$$
  echo 'Navigation stack is ready.'
}

for node in \
  /raicom_supervisor \
  /raicom_vision \
  /local_voice_node \
  /face_detection \
  /face_rec_service \
  /head_camera \
  /service_group_welcome_mission \
  /service_group_patrol_mission \
  /home_service_mission
do
  rosnode kill "${node}" >/dev/null 2>&1 || true
done

case "${TASK}" in
  provincial_welcome|selection_welcome)
    require_navigation_ready
    exec roslaunch raicom_supervisor selection_welcome_stack.launch
    ;;
  provincial_patrol|selection_patrol)
    require_navigation_ready
    exec roslaunch raicom_supervisor selection_patrol_stack.launch
    ;;
  provincial_patrol_scripted|selection_patrol_scripted)
    require_navigation_ready
    exec roslaunch raicom_supervisor selection_patrol_stack.launch       config_file:="$(rospack find service_group_mission)/config/provincial_patrol_test.yaml"
    ;;
  home_auto)
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch mission_type:=auto
    ;;
  home_guide)
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch mission_type:=guide
    ;;
  home_assistant)
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch mission_type:=assistant
    ;;
  home_find)
    if [[ "${TARGET_OBJECT}" != "cell phone" && "${TARGET_OBJECT}" != "backpack" && "${TARGET_OBJECT}" != "water bottle" ]]; then
      echo "home_find target must be: cell phone, backpack, or water bottle"
      exit 2
    fi
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch \
      mission_type:=find_object \
      target_object:="${TARGET_OBJECT}"
    ;;
  *)
    echo "Unknown task: ${TASK}"
    exit 2
    ;;
esac
