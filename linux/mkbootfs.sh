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

# Create a boot filesystem from scratch
# This script requires a cross compiling environment to be created first
# See usage for a description of the arguments

. ./functions.sh

BLDPKGS="$BLDPKGS linux"
BUILDOPTS="$BUILDOPTS -j2"

init "create a boot filesystem from scratch" "PKGn" "$BLDPKGS" \
  "list of packages to be added to the image"

set_arg "--image|-i" "" "IMAGE" "bootfs.img" \
  "boot filesystem image to be created"
set_arg "--root" "DIR" "FSROOT" ".bootfs.root" \
  "temporary root of filesystem"
set_arg "--build-root" "DIR" "BUILDROOT" ".bootfs.build" \
  "temporary build root"
set_arg "--mount-point" "DIR" "MNT" ".rootfs.mount" \
  "temporary mount point of root filesystem"
set_arg "--type" "ext2|ext3|..." "FSTYPE" "ext2" \
  "type of filesystem to be built"
set_arg "--xcompile-root" "DIR" "XCROOT" ".xcomp.root" \
  "root directory of the cross compiler"
set_arg "--host" "i686|powerpc|..." "HOST" "`uname -m`" \
  "override host architecture"
set_arg "--target" "i686|powerpc|..." "TARGET" "powerpc" \
  "target architecture"
set_arg "--package-dir" "DIR" "PKGDIR" "packages" \
  "directory containing packages"
set_arg "--config-dir" "DIR" "CFGDIR" "configurations" \
  "directory containing build configurations"
set_arg "--no-build" "" "NOBUILD" "false" \
  "do not build and install any packages"
set_arg "--install" "" "INSTALL" "false" \
  "perform install stage only"
set_arg "--clean" "" "CLEAN" "false" \
  "remove working directories"

check_args $*
check_uid

abs_path $FSROOT
FSROOT=$ABSPATH
abs_path $XCROOT
XCROOT=$ABSPATH
PATH="$XCROOT/bin:$PATH"

message "making boot filesystem image $IMAGE"

check_xcomp $TARGET gcc g++ ar as ranlib ld strip

execute "mkdir -p $FSROOT"
execute "mkdir -p $BUILDROOT"

set_xcomp $TARGET $XCROOT $FSROOT true

if [ "$NOBUILD" != "true" ]; then
  build_packages bootfs $FSROOT "" $BUILDROOT "$BUILDOPTS" $PKGDIR $CFGDIR \
    $HOST $TARGET $INSTALL $PKGn
fi

mk_image $IMAGE $FSROOT $MNT $FSTYPE 0

if [ "$CLEAN" == "true" ]; then
  clean $BUILDROOT $FSROOT $LOGFILE
else
  clean $LOGFILE
fi

get_filesize $IMAGE
message "success, size of the boot filesystem is ${SIZE}kB"