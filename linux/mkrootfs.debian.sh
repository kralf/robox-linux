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

# Create a Debian root filesystem from scratch
# See usage for a description of the arguments

. ./functions.sh

init "create a Debian root filesystem from scratch" "PKG" "" \
  "list of packages to be added to the image"

set_arg "--image|-i" "FILE" "IMAGE" "rootfs.debian.img" \
  "root filesystem image to be created"
set_arg "--size" "SIZE" "SIZE" "128" \
  "size of the root filesystem in MB"
set_arg "--target" "i686|powerpc|..." "TARGET" "powerpc" \
  "target architecture"
set_arg "--dist" "woody|sarge|..." "DIST" "sarge" \
  "Debian distribution"
set_arg "--mirror" "us|de|..." "MIRROR" "de" \
  "package download mirror"
set_arg "--mount-point" "DIR" "MNT" ".rootfs.debian.mount" \
  "temporary mount point of root filesystem"

check_args $*
check_uid

SERVER="http://ftp.$MIRROR.debian.org/debian"
if [ ${#PKG[@]} != 0 ]; then
  PKGS=${PKG[@]}
  PKGS=${PKGS//" "/","}
  INCPKGS="--include=$PKGS"
fi

message "making Debian root filesystem image $IMAGE"
stage_up

message "creating zeroed root filesystem image $IMAGE"
execute "dd if=/dev/zero of=$IMAGE bs=1M count=$SIZE"
execute "/sbin/mkfs -F $IMAGE"

message "mounting root filesystem image to $MNT"
execute "mkdir $MNT"
execute "mount -o loop $IMAGE $MNT"

message "processing package list"
execute "debootstrap --verbose --arch $TARGET $INCPKGS $DIST $MNT $SERVER"

message "unmounting root filesystem image"
execute "umount $MNT"
execute "rm -r $MNT"

stage_down
message "success"
