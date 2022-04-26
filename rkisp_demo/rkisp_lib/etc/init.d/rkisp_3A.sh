#!/bin/bash
#
# Start 3A daemon for each /dev/mediaX device
#

start_3A()
{
    i=0
    for MEDIA_DEV in $(ls -1 /dev/media*); do
      /usr/bin/rkisp_3A_server --mmedia=$MEDIA_DEV 2>&1 | logger -t rkisp_3A-$i &
      let i++
    done
}

stop_3A()
{
    killall rkisp_3A_server
}

case "$1" in
  start)
    start_3A
    ;;
  stop)
    stop_3A
    ;;
  reload)
    ;;
  *)
    echo "Usage: $0 {start|stop}"
    exit 1
    ;;
esac
exit 0
