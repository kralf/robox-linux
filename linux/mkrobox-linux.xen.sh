#!/bin/bash
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

# Create robox-linux image  and boot it in a Xen virtual machine
# See usage for a description of the arguments

. ubash

XCPKGS="linux-xen-headers binutils gcc-min glibc gcc"
RFSPKGS="coreutils glibc-min ncurses readline bash pcre grep sed zlib"
RFSPKGS="$RFSPKGS sysvinit e2fsprogs util-linux module-init-tools udev"
RFSPKGS="$RFSPKGS procps hostname sysklogd shadow linux-xen-modules"
BRDPKGS="linux-xen"

script_init "Create robox-linux image and boot it in a Xen virtual machine"

script_setopt "--image-root" "DIR" "XENIMGROOT" "images" \
  "root directory of the images to be created"
script_setopt "--host" "i686|powerpc|..." "XENHOST" "`uname -m`" \
  "override host architecture"
script_setopt "--target" "i686|powerpc|..." "XENTARGET" "i686" \
  "target architecture"

script_setopt "--package-dir" "DIR" "XENPKGDIR" "pkg" \
  "directory containing packages"
script_setopt "--config-dir" "DIR" "XENCFGDIR" "conf" \
  "directory containing build configurations"

script_setopt "--no-xcomp" "" "XENNOXCOMP" "false" \
  "do not build and install cross compiler packages"
script_setopt "--no-rootfs" "" "XENNOROOT" "false" \
  "do not build and install root filesystem packages"
script_setopt "--no-bootfs" "" "XENNOBOOT" "false" \
  "do not build and install boot filesystem packages"
script_setopt "--no-build" "" "XENNOBUILD" "false" \
  "do not build and install any packages"
script_setopt "--install" "" "XENINSTALL" "false" \
  "perform install stage only"
script_setopt "--boot" "" "XENBOOT" "false" \
  "just boot into the image"

script_setopt "--mount-point" "DIR" "XENMNT" ".bootfs.mount" \
  "mount point of boot filesystem"
script_setopt "--xen-config" "FILE" "XENCFG" "robox-linux.xen" \
  "Xen virtual machine configuration"
script_setopt "--clean" "" "XENCLEAN" "false" \
  "remove working directories"

script_checkopts $*
script_checkroot

fs_abspath $XENIMGROOT XENIMGROOT

true VERBOSE && XENARGS="--verbose"
XENARGS="$XENARGS --host $XENHOST --target $XENTARGET"
XENARGS="$XENARGS --package-dir $XENPKGDIR --config-dir $XENCFGDIR"

true XENNOBUILD && XENARGS="$XENARGS --no-build"
true XENINSTALL && XENARGS="$XENARGS --install"
true XENCLEAN && XENARGS="$XENARGS --clean"

true XENNOXCOMP && XCARGS="--no-build"

RFSARGS="--image-root $XENIMGROOT --no-exclusions"
true XENNOROOT && RFSARGS="$RFSARGS --no-build"

BRDARGS="--image-root $XENIMGROOT"
true XENNOBOOT && BRDARGS="$BRDARGS --no-build"

message_start "making robox-linux"

if false XENBOOT; then
  ./mkxcomp.sh $XENARGS $XCARGS $XCPKGS && \
  ./mkrootfs.sh $XENARGS $RFSARGS $RFSPKGS && \
  ./mkbootfs.sh $XENARGS $BRDARGS $BRDPKGS
fi

if [ $? == 0 ]; then
  fs_abspath $XENMNT XENMNT

  ./mountfs.sh --image $XENIMGROOT/bootfs.img --mount-point $XENMNT boot
  xen_bootimg "robox-linux" $XENCFG $XENMNTPATH "$XENIMGROOT/rootfs.img"
  ./umountfs.sh --mount-point $XENMNTPATH boot

  log_clean
fi
