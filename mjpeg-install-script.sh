#!/bin/bash
cp installer.py /tmp
cd /tmp
if [ ! -d opkg_cache ] || [ ! -d plp_cache ] ; then
    python3 installer.py download-opkg mjpg-streamer
fi

python3 installer.py install-opkg mjpg-streamer
