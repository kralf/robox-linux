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

# Create a boot ramdisk from scratch
# This script requires a cross compiling environment to be created first
# See usage for a description of the arguments

. functions/global.sh

BRDPKGS="linux"

BRDCPDEVS="console initctl kmsg mem null ram[0-6] rtc tty[0-6] ttyS[0-3]"

BRDRFSCHOWN="/=root:root"
BRDRFSCHMOD=""

script_init "create a boot ramdisk from scratch" "PKGn" "$BRDPKGS" \
  "list of packages to be added to the boot ramdisk"

script_setopt "--build-root" "DIR" "BRDBUILDROOT" ".bootrd.build" \
  "temporary build root"
script_setopt "--image-root" "DIR" "BRDIMGROOT" ".bootrd.images" \
  "boot ramdisk image root directory"
script_setopt "--xcomp-root" "DIR" "BRDXCROOT" ".xcomp.root" \
  "root directory of the cross compiler"
script_setopt "--rootfs-root" "DIR" "BRDRFSROOT" ".rootfs.root" \
  "root directory of the root filesystem"

script_setopt "--rootfs-mount-point" "DIR" "BRDRFSMOUNTPOINT" ".rootfs.mount" \
  "temporary mount point of root filesystem"
script_setopt "--rootfs-type" "ext2|ext3|..." "BRDRFSTYPE" "ext2" \
  "type of root filesystem to be built"
script_setopt "--rootfs-block-size" "BYTES" "BRDRFSBLOCKSIZE" "1024" \
  "size of root filesystem blocks in bytes"
script_setopt "--rootfs-space" "BLOCKS" "BRDRFSSPACE" "128" \
  "number of free root filesystem blocks"

script_setopt "--host" "i686|powerpc|..." "BRDHOST" "`uname -m`" \
  "override host architecture"
script_setopt "--target" "i686|powerpc|..." "BRDTARGET" "`uname -m`" \
  "target architecture"
script_setopt "--cores" "NUM" "BRDCORES" "1" "number of cores to compile on"
script_setopt "--make-args|-m" "ARGS" "BRDMAKEARGS" "" \
  "additional arguments to be passed to make"

script_setopt "--package-dir" "DIR" "BRDPKGDIR" "packages" \
  "directory containing packages"
script_setopt "--config-dir" "DIR" "BRDCONFDIR" "configurations" \
  "directory containing build configurations"
script_setopt "--patch-dir" "DIR" "BRDPATCHDIR" "patches" \
  "directory containing patches"

script_setopt "--no-build" "" "BRDNOBUILD" "false" \
  "do not build and install any packages"
script_setopt "--clean" "" "BRDCLEAN" "false" "remove working directories"

script_checkopts $*
script_checkroot

BRDBUILDROOT="$BRDBUILDROOT/$BRDTARGET"
BRDIMGROOT="$BRDIMGROOT/$BRDTARGET"
fs_abspath "$BRDXCROOT/$BRDTARGET" BRDXCROOT
fs_abspath "$BRDRFSROOT/$BRDTARGET" BRDRFSROOT
BRDMAKEOPTS="$BRDMAKEOPTS -j$BRDCORES"

PATH="$PATH:$BRDXCROOT/bin"

message_boldstart "making boot ramdisk in $BRDROOT"

build_checktools $BRDTARGET gcc g++ ar as ranlib ld strip
build_setenv $BRDXCROOT $BRDTARGET $BRDDEBUG

[ -d "$BRDBUILDROOT" ] || execute "mkdir -p $BRDBUILDROOT"
[ -d "$BRDIMGROOT" ] || execute "mkdir -p $BRDIMGROOT"
fs_abspath $BRDBUILDROOT BRDBUILDROOT
fs_abspath $BRDIMGROOT BRDIMGROOT

BRDIMG="$BRDIMGROOT/bootrd.img"
BRDRFSIMG="$BRDIMGROOT/rootfs.img"

if [ -d "$BRDRFSROOT" ]; then
  fs_mkimg $BRDRFSIMG $BRDRFSROOT $BRDRFSMOUNTPOINT $BRDRFSTYPE \
    $BRDRFSBLOCKSIZE $BRDRFSSPACE

  fs_mountimg $BRDRFSIMG $BRDRFSMOUNTPOINT
  fs_chowndirs $BRDRFSMOUNTPOINT "$BRDRFSCHOWN"
  fs_chmoddirs $BRDRFSMOUNTPOINT "$BRDRFSCHMOD"
  fs_cpdevices $BRDRFSMOUNTPOINT $BRDCPDEVS
  fs_umountimg $BRDRFSIMG $BRDRFSMOUNTPOINT
else
  message_exit "root filesystem seems to be missing"
fi

if false BRDNOBUILD; then
  build_packages "bootrd" $BRDPKGDIR $BRDCONFDIR $BRDPATCHDIR $BRDBUILDROOT \
    $BRDBUILDROOT $BRDHOST $BRDTARGET "$BRDMAKEOPTS" false $PKGn
fi

if true BRDCLEAN; then
  message_start "cleaning up working directories"
  execute "rm -rf $BRDBUILDROOT"
  message_end
fi

fs_getfilesize $BRDIMG BRDIMGSIZE
message_boldend "success, size of the boot ramdisk is ${BRDIMGSIZE}kB"

log_clean
