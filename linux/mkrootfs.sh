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

# Create a root filesystem from scratch
# This script requires a cross compiling environment to be created first
# See usage for a description of the arguments

. functions/global.sh

RFSPKGS="coreutils glibc-min ncurses readline bash pcre grep sed"
RFSPKGS="$RFSPKGS zlib sysvinit e2fsprogs util-linux module-init-tools"
RFSPKGS="$RFSPKGS udev procps hostname sysklogd shadow dpkg linux-modules"
RFSPKGS="$RFSPKGS net-tools iputils ifupdown debianutils openssl openssh"
RFSPKGS="$RFSPKGS robox-drivers robox-linux carmen"

RFSMKDIRS="/proc /sys /dev /mnt /etc /boot /home /root /tmp /usr"
RFSMKDIRS="$RFSMKDIRS /var/lock /var/log /var/mail /var/run /var/spool"
RFSMKFILES="/var/run/utmp /var/log/lastlog /var/log/wtmp /var/log/btmp"

RFSEXDIRS="/man /share /usr/man /usr/info /usr/share /usr/include"
RFSEXFILES="*.a *.o *.old"

script_init "create a root filesystem from scratch" "PKGn" "$RFSPKGS" \
  "list of packages to be added to the root filesytem"

script_setopt "--root" "DIR" "RFSROOT" ".rootfs.root" \
  "temporary root of filesystem"
script_setopt "--build-root" "DIR" "RFSBUILDROOT" ".rootfs.build" \
  "temporary build root"
script_setopt "--xcomp-root" "DIR" "RFSXCROOT" ".xcomp.root" \
  "root directory of the cross compiler"

script_setopt "--host" "i686|powerpc|..." "RFSHOST" "`uname -m`" \
  "override host architecture"
script_setopt "--target" "i686|powerpc|..." "RFSTARGET" "`uname -m`" \
  "target architecture"
script_setopt "--cores" "NUM" "RFSCORES" "1" "number of cores to compile on"
script_setopt "--make-args|-m" "ARGS" "RFSMAKEARGS" "" \
  "additional arguments to be passed to make"

script_setopt "--package-dir" "DIR" "RFSPKGDIR" "packages" \
  "directory containing packages"
script_setopt "--config-dir" "DIR" "RFSCONFDIR" "configurations" \
  "directory containing build configurations"
script_setopt "--patch-dir" "DIR" "RFSPATCHDIR" "patches" \
  "directory containing patches"

script_setopt "--no-build" "" "RFSNOBUILD" "false" \
  "do not build and install any packages"
script_setopt "--debug" "" "RFSDEBUG" "false" \
  "emit debugging information for all symbols"
script_setopt "--no-strip" "" "RFSNOSTRIP" "false" \
  "do not strip symbols from binary objects"
script_setopt "--no-excludes" "" "RFSNOEXCLUDES" "false" \
  "do not exclude any files from filesystem"
script_setopt "--install" "" "RFSINSTALL" "false" "perform install stage only"
script_setopt "--clean" "" "RFSCLEAN" "false" "remove working directories"

script_checkopts $*

RFSROOT="$RFSROOT/$RFSTARGET"
RFSBUILDROOT="$RFSBUILDROOT/$RFSTARGET"
fs_abspath "$RFSXCROOT/$RFSTARGET" RFSXCROOT
RFSMAKEOPTS="$RFSMAKEOPTS -j$RFSCORES"

PATH="$PATH:$RFSXCROOT/bin"

message_boldstart "making root filesystem in $RFSROOT"

build_checktools $RFSTARGET gcc g++ ar as ranlib ld strip
build_setenv $RFSXCROOT $RFSTARGET $RFSDEBUG

[ -d "$RFSROOT" ] || execute "mkdir -p $RFSROOT"
[ -d "$RFSBUILDROOT" ] || execute "mkdir -p $RFSBUILDROOT"
fs_abspath $RFSROOT RFSROOT
fs_abspath $RFSBUILDROOT RFSBUILDROOT

[ -n "$RFSMKDIRS" ] && fs_mkdirs $RFSROOT $RFSMKDIRS
[ -n "$RFSMKFILES" ] && fs_mkfiles $RFSROOT $RFSMKFILES

if false RFSNOBUILD; then
  build_packages "rootfs" $RFSPKGDIR $RFSCONFDIR $RFSPATCHDIR $RFSBUILDROOT \
    $RFSROOT $RFSHOST $RFSTARGET "$RFSMAKEOPTS" $RFSINSTALL ${PKGn[@]}
fi

if false RFSNOSTRIP; then
  build_stripsyms $RFSROOT
fi
if false RFSNOEXCLUDES; then
  fs_rmdirs $RFSROOT $RFSEXDIRS
  fs_rmfiles $RFSROOT $RFSEXFILES
fi

if true RFSCLEAN; then
  message_start "cleaning up working directories"
  execute "rm -rf $RFSBUILDROOT"
  message_end
fi

fs_getdirsize $RFSROOT RFSSIZE
message_boldend "success, size of the root filesystem is ${RFSSIZE}kB"

log_clean
