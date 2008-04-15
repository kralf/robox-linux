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

# Create a root filesystem from scratch
# This script requires a cross compiling environment to be created first
# See usage for a description of the arguments

. ./functions.sh

BLDPKGS="$BLDPKGS coreutils glibc-min ncurses readline bash pcre grep sed"
BLDPKGS="$BLDPKGS zlib sysvinit e2fsprogs util-linux module-init-tools udev"
BLDPKGS="$BLDPKGS procps hostname sysklogd shadow dpkg linux-modules"
BLDPKGS="$BLDPKGS net-tools iputils ifupdown debianutils openssl openssh"
MAKEOPTS="$MAKEOPTS"
MKDIRS="$MKDIRS proc sys dev mnt etc boot home root tmp usr"
MKDIRS="$MKDIRS var var/lock var/log var/mail var/run var/spool"
MKFILES="$MKFILES var/run/utmp 664"
MKFILES="$MKFILES var/log/lastlog 664 var/log/wtmp 664 var/log/btmp 600"
MKDEVS="$MKDEVS console kmsg mem null ram[0-6] tty[0-6] ttyS[0-3] initctl rtc"
EXCLDIRS="$EXCLDIRS usr/man usr/info usr/share usr/include"
EXCLFILES="$EXCLFILES *.a *.o"

init "create a root filesystem from scratch" "PKGn" "$BLDPKGS" \
  "list of packages to be added to the image"

set_arg "--root" "DIR" "FSROOT" ".rootfs.root" \
  "temporary root of filesystem"
set_arg "--image-root" "DIR" "IMGROOT" "images" \
  "root directory of the images to be created"
set_arg "--build-root" "DIR" "BUILDROOT" ".rootfs.build" \
  "temporary build root"
set_arg "--mount-point" "DIR" "MNT" ".rootfs.mount" \
  "temporary mount point of root filesystem"
set_arg "--type" "ext2|ext3|..." "FSTYPE" "ext2" \
  "type of filesystem to be built"
set_arg "--block-size" "BYTES" "BLOCKSIZE" "1024" \
  "size of filesystem blocks in bytes"
set_arg "--space" "BLOCKS" "FSSPACE" "128" \
  "number of free filesystem blocks"
set_arg "--xcompile-root" "DIR" "XCROOT" ".xcomp.root" \
  "root directory of the cross compiler"
set_arg "--host" "i686|powerpc|..." "HOST" "`uname -m`" \
  "override host architecture"
set_arg "--target" "i686|powerpc|..." "TARGET" "powerpc" \
  "target architecture"
set_arg "--cores" "NUM" "CORES" "1" \
  "number of cores to compile on"
set_arg "--package-dir" "DIR" "PKGDIR" "packages" \
  "directory containing packages"
set_arg "--patch-dir" "DIR" "PATCHDIR" "patches" \
  "directory containing patches"
set_arg "--config-dir" "DIR" "CFGDIR" "configurations" \
  "directory containing build configurations"
set_arg "--sysinit-dir" "DIR" "SYSINITDIR" "sysinit" \
  "directory containing sysinit files"
set_arg "--debug" "" "DEBUG" "false" \
  "emit debugging information for all symbols"
set_arg "--no-arch-split" "" "NOARCH" "false" \
  "do not split by architecture dependency"
set_arg "--no-build" "" "NOBUILD" "false" \
  "do not build and install any packages"
set_arg "--install" "" "INSTALL" "false" \
  "perform install stage only"
set_arg "--no-exclusions" "" "NOEXCLUDE" "false" \
  "do not exclude any files from image"
set_arg "--clean" "" "CLEAN" "false" \
  "remove working directories"

check_args $*
check_uid

abs_path $FSROOT
FSROOT="$ABSPATH/$TARGET"
FSUSRDIR=""
if [ "$NOARCH" != "true" ]; then
  FSUSRDIR="/usr"
fi
abs_path $IMGROOT
IMGROOT="$ABSPATH/$TARGET"
IMAGE="$IMGROOT/rootfs.img"
abs_path $BUILDROOT
BUILDROOT="$ABSPATH/$TARGET"
abs_path $XCROOT
XCROOT="$ABSPATH/$TARGET"
PATH="$XCROOT/bin:$PATH"
MAKEOPTS="$MAKEOPTS -j$CORES"

message "making root filesystem image $IMAGE"

check_xcomp $TARGET gcc g++ ar as ranlib ld strip

execute "mkdir -p $FSROOT"
execute "mkdir -p $IMGROOT"
execute "mkdir -p $BUILDROOT"

set_xcomp $TARGET $XCROOT $FSROOT $DEBUG

stage_up
message "creating directory structure in $FSROOT"

mk_dirs $FSROOT $MKDIRS
mk_files $FSROOT $MKFILES
mk_devices $FSROOT $MKDEVS

stage_down

if [ "$NOBUILD" != "true" ]; then
  build_packages rootfs $FSROOT $FSUSRDIR $BUILDROOT "$MAKEOPTS" $PKGDIR \
    $PATCHDIR $CFGDIR $HOST $TARGET $INSTALL $PKGn
fi

cp_files $SYSINITDIR $FSROOT/etc
chown_dirs $FSROOT etc root:root

if [ "$NOEXCLUDE" != "true" ]; then
  rm_dirs $FSROOT $EXCLDIRS
  rm_files $FSROOT $EXCLFILES
fi
rm_brokenlinks $FSROOT

mk_image $IMAGE $FSROOT $MNT $FSTYPE $BLOCKSIZE $FSSPACE

# if [ "$CLEAN" == "true" ]; then
#   clean $BUILDROOT $FSROOT $LOGFILE
# else
#   clean $LOGFILE
# fi

get_dirsize $FSROOT
message "success, size of the root filesystem is ${SIZE}kB"
