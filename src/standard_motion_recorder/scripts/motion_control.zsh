#!/usr/bin/env zsh
set -e

usage() {
  print "Usage:"
  print "  motion_control.zsh start"
  print "  motion_control.zsh pause|resume|status"
  print "  motion_control.zsh stop [file.json]"
  print "  motion_control.zsh play <file.json> [speed] [loop:true|false]"
  print "  motion_control.zsh stop-playback"
}

command=${1:-}
case ${command} in
  start)
    ros2 service call /motion/record/start bw_interface/srv/StartRecord "{}"
    ;;
  pause)
    ros2 service call /motion/record/pause bw_interface/srv/PauseRecord "{}"
    ;;
  resume)
    ros2 service call /motion/record/resume bw_interface/srv/ResumeRecord "{}"
    ;;
  stop)
    ros2 service call /motion/record/stop bw_interface/srv/StopRecord \
      "{file_path: '${2:-}'}"
    ;;
  status)
    ros2 service call /motion/status bw_interface/srv/GetStatus "{}"
    ;;
  play)
    [[ -n ${2:-} ]] || { usage; exit 2; }
    ros2 service call /motion/playback/start bw_interface/srv/PlaybackStart \
      "{file_path: '${2}', speed: ${3:-1.0}, loop: ${4:-false}}"
    ;;
  stop-playback)
    ros2 service call /motion/playback/stop bw_interface/srv/PlaybackStop "{}"
    ;;
  *)
    usage
    exit 2
    ;;
esac
