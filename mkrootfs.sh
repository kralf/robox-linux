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

DEFPKGS="coreutils glibc-min ncurses readline bash pcre grep sed zlib sysvinit"
DEFPKGS="$DEFPKGS e2fsprogs util-linux module-init-tools udev linux-modules"
DEFPKGS="$DEFPKGS robox-linux"
DIRS="proc dev mnt etc boot home root tmp usr"
DIRS="$DIRS var var/lock var/log var/mail var/run var/spool"
FILES="etc/passwd 600 etc/group 600"
FILES="$FILES var/run/utmp 664"
FILES="$FILES var/log/lastlog 664 var/log/wtmp 664 var/log/btmp 600"
FILECONTENT="etc/passwd root::0:0:root:/root:/bin/bash"
FILECONTENT="$FILECONTENT etc/group"
FILECONTENT="$FILECONTENT root:x:0:\nbin:x:1:\nsys:x:2:\nkmem:x:3:\ntty:x:4:\n"
FILECONTENT="${FILECONTENT}tape:x:5:\ndaemon:x:6:\nfloppy:x:7:\ndisk:x:8:\n"
FILECONTENT="${FILECONTENT}lp:x:9:\ndialout:x:10:\naudio:x:11:\nvideo:x:12:\n"
FILECONTENT="${FILECONTENT}utmp:x:13:\nusb:x:14:\ncdrom:x:15:"
DEVS="console kmsg mem null ram[0-6] tty[0-6] ttyS[0-3] initctl"
EXCLDIRS="usr/man usr/info usr/share usr/include"
EXCLFILES="*.a *.o"

init "create a root filesystem from scratch" "PKG" "$DEFPKGS" \
  "list of packages to be added to the image"

set_arg "--image|-i" "" "IMAGE" "rootfs.img" \
  "root filesystem image to be created"
set_arg "--root" "DIR" "FSROOT" ".rootfs.root" \
  "temporary root of filesystem"
set_arg "--build-root" "DIR" "BUILDROOT" ".rootfs.build" \
  "temporary build root"
set_arg "--mount-point" "DIR" "MNT" ".rootfs.mount" \
  "temporary mount point of root filesystem"
set_arg "--type" "ext2|ext3|..." "FSTYPE" "ext2" \
  "type of filesystem to be built"
set_arg "--space" "BLOCKS" "FSSPACE" "128" \
  "number of free 1kB filesystem blocks"
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
FSROOT=$ABSPATH
FSUSRDIR=""
if [ "$NOARCH" != "true" ]; then
  FSUSRDIR="/usr"
fi
abs_path $XCROOT
XCROOT=$ABSPATH
PATH="$PATH:$XCROOT/bin"

message "making root filesystem image $IMAGE"

check_xcomp $TARGET gcc g++ ar as ranlib ld strip

execute "mkdir -p $FSROOT"
execute "mkdir -p $BUILDROOT"

set_xcomp $TARGET $XCROOT $FSROOT $DEBUG

stage_up
message "creating directory structure in $FSROOT"

mk_dirs $FSROOT $DIRS
mk_files $FSROOT $FILES
fill_files $FSROOT $FILECONTENT
mk_devices $FSROOT $DEVS

stage_down

if [ "$NOBUILD" != "true" ]; then
  process_packages rootfs $FSROOT $FSUSRDIR $BUILDROOT $PKGDIR $CFGDIR \
    $HOST $TARGET $INSTALL $PKG
fi

if [ "$NOEXCLUDE" != "true" ]; then
  rm_dirs $FSROOT $EXCLDIRS
  rm_files $FSROOT $EXCLFILES
fi
rm_brokenlinks $FSROOT

mk_image $IMAGE $FSROOT $MNT $FSTYPE $FSSPACE

if [ "$CLEAN" == "true" ]; then
  clean $BUILDROOT $FSROOT $LOGFILE
else
  clean $LOGFILE
fi

get_dirsize $FSROOT
message "success, size of the root filesystem is ${SIZE}kB"
