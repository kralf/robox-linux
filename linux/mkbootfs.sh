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

# Create a boot filesystem from scratch
# This script requires a cross compiling environment to be created first
# See usage for a description of the arguments

. functions/global.sh

BLDPKGS="$BLDPKGS linux"
MAKEOPTS="$MAKEOPTS"

script_init "create a boot filesystem from scratch" "PKGn" "$BLDPKGS" \
  "list of packages to be added to the image"

script_setopt "--root" "DIR" "FSROOT" ".bootfs.root" \
  "temporary root of filesystem"
script_setopt "--image-root" "DIR" "IMGROOT" "images" \
  "root directory of the images"
script_setopt "--build-root" "DIR" "BUILDROOT" ".bootfs.build" \
  "temporary build root"
script_setopt "--mount-point" "DIR" "MNT" ".rootfs.mount" \
  "temporary mount point of root filesystem"
script_setopt "--type" "ext2|ext3|..." "FSTYPE" "ext2" \
  "type of filesystem to be built"
script_setopt "--block-size" "BYTES" "BLOCKSIZE" "1024" \
  "size of filesystem blocks in bytes"
script_setopt "--xcompile-root" "DIR" "XCROOT" ".xcomp.root" \
  "root directory of the cross compiler"
script_setopt "--host" "i686|powerpc|..." "HOST" "`uname -m`" \
  "override host architecture"
script_setopt "--target" "i686|powerpc|..." "TARGET" "powerpc" \
  "target architecture"
script_setopt "--cores" "NUM" "CORES" "1" "number of cores to compile on"
script_setopt "--package-dir" "DIR" "PKGDIR" "packages" \
  "directory containing packages"
script_setopt "--patch-dir" "DIR" "PATCHDIR" "patches" \
  "directory containing patches"
script_setopt "--config-dir" "DIR" "CFGDIR" "configurations" \
  "directory containing build configurations"
script_setopt "--no-build" "" "NOBUILD" "false" \
  "do not build and install any packages"
script_setopt "--install" "" "INSTALL" "false" "perform install stage only"
script_setopt "--clean" "" "CLEAN" "false" "remove working directories"

script_checkopts $*

abs_path $FSROOT
FSROOT="$ABSPATH/$TARGET"
abs_path $IMGROOT
IMGROOT="$ABSPATH/$TARGET"
IMAGE="$IMGROOT/bootfs.img"
abs_path $BUILDROOT
BUILDROOT="$ABSPATH/$TARGET"
abs_path $XCROOT
XCROOT="$ABSPATH/$TARGET"
PATH="$XCROOT/bin:$PATH"
MAKEOPTS="$MAKEOPTS -j$CORES"

message_boldstart "making boot filesystem image $IMAGE"

check_xcomp $TARGET gcc g++ ar as ranlib ld strip

execute "mkdir -p $FSROOT"
execute "mkdir -p $BUILDROOT"

set_xcomp $TARGET $XCROOT $FSROOT true

if [ "$NOBUILD" != "true" ]; then
  build_packages bootfs $FSROOT "" $BUILDROOT "$MAKEOPTS" $PKGDIR $PATCHDIR \
    $CFGDIR $HOST $TARGET $INSTALL $PKGn
fi

mk_image $IMAGE $FSROOT $MNT $FSTYPE $BLOCKSIZE 0

if [ "$CLEAN" == "true" ]; then
  clean $BUILDROOT $FSROOT $LOGFILE
else
  clean $LOGFILE
fi

get_filesize $IMAGE
message "success, size of the boot filesystem is ${SIZE}kB"
