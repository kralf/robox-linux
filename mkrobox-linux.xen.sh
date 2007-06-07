############################################################################
#    Copyright (C) 2007 by Ralf 'Decan' Kaestner                           #
#    ralf.kaestner@gmail.com                                               #
#                                                                          #
#    This program is free software; you can redistribute it and#or modify  #
#    it under the terms of the GNU General Public License as published by  #
#    the Free Software Foundation; either version 2 of the License, or     #
#    (at your option) any later version.                                   #
#                                                                          #
#    This program is distributed in the hope that it will be useful,       #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#    GNU General Public License for more details.                          #
#                                                                          #
#    You should have received a copy of the GNU General Public License     #
#    along with this program; if not, write to the                         #
#    Free Software Foundation, Inc.,                                       #
#    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             #
############################################################################

#!/bin/bash

# Create robox-linux image  and boot it in a Xen virtual machine
# See usage for a description of the arguments

. ./functions.sh

init "create robox-linux image and boot it in a Xen virtual machine"

set_arg "--host" "i686|powerpc|..." "HOST" "`uname -m`" \
  "override host architecture"
set_arg "--target" "i686|powerpc|..." "TARGET" "powerpc" \
  "target architecture"
set_arg "--package-dir" "DIR" "PKGDIR" "packages" \
  "directory containing packages"
set_arg "--config-dir" "DIR" "CFGDIR" "configurations" \
  "directory containing build configurations"
set_arg "--tftp-server" "ADDRESS" "TFTPSERVER" "shantipc1" \
  "ip or dns name of the tftp server"
set_arg "--tftp-user" "LOGIN" "TFTPUSER" "tftp" \
  "login name of the tftp user"
set_arg "--tftp-root" "DIR" "TFTPROOT" "~" \
  "tftp root directory"
set_arg "--no-build" "" "NOBUILD" "false" \
  "do not build and install any packages"
set_arg "--install" "" "INSTALL" "false" \
  "perform install stage only"
set_arg "--clean" "" "CLEAN" "false" \
  "remove working directories"
set_arg "--mount-point" "DIR" "MNT" ".bootfs.mount" \
  "mount point of boot filesystem"
set_arg "--xen-config" "FILE" "XENCFG" "robox-linux.xen" \
  "Xen virtual machine configuration"
set_arg "--boot" "" "BOOT" "false" \
  "just boot into the image"


check_args $*
check_uid

STAGE=0
export STAGE

CALLARGS="--host $HOST --target $TARGET"
CALLARGS="$CALLARGS --package-dir $PKGDIR --config-dir $CFGDIR"

if [ "$NOBUILD" == "true" ]; then
  CALLARGS="$CALLARGS --no-build"
fi
if [ "$INSTALL" == "true" ]; then
  CALLARGS="$CALLARGS --install"
fi
if [ "$CLEAN" == "true" ]; then
  CALLARGS="$CALLARGS --clean"
fi

message "making robox-linux"
stage_up

if [ "$BOOT" != "true" ]; then
  ./mkxcomp.sh $CALLARGS && ./mkrootfs.sh $CALLARGS && \
    ./mkbootfs.sh $CALLARGS linux-xen
fi

if [ $? = 0 ]; then
  abs_path $MNT
  MNTPATH="$ABSPATH"
  ./mountbootfs.sh --mount-point $MNTPATH

  FAILEXIT="false"  
  boot_image "robox-linux" $XENCFG $MNTPATH "rootfs.img"
  FAILEXIT="true"
  
  ./umountbootfs.sh --mount-point $MNTPATH
  stage_down
  
  if [ "$RETVAL" = 0 ]; then
    #clean $LOGFILE
    message "success"
  fi
fi
