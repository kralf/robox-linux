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

BFSPKGS="linux"

script_init "create a boot filesystem from scratch" "PKGn" "$BFSPKGS" \
  "list of packages to be added to the boot filesystem"

script_setopt "--root" "DIR" "BFSROOT" ".bootfs.root" \
  "temporary root of filesystem"
script_setopt "--build-root" "DIR" "BFSBUILDROOT" ".bootfs.build" \
  "temporary build root"
script_setopt "--xcomp-root" "DIR" "BFSXCROOT" ".xcomp.root" \
  "root directory of the cross compiler"

script_setopt "--host" "i686|powerpc|..." "BFSHOST" "`uname -m`" \
  "override host architecture"
script_setopt "--target" "i686|powerpc|..." "BFSTARGET" "`uname -m`" \
  "target architecture"
script_setopt "--cores" "NUM" "BFSCORES" "1" "number of cores to compile on"
script_setopt "--make-args|-m" "ARGS" "BFSMAKEARGS" "" \
  "additional arguments to be passed to make"

script_setopt "--package-dir" "DIR" "BFSPKGDIR" "packages" \
  "directory containing packages"
script_setopt "--config-dir" "DIR" "BFSCONFDIR" "configurations" \
  "directory containing build configurations"
script_setopt "--patch-dir" "DIR" "BFSPATCHDIR" "patches" \
  "directory containing patches"

script_setopt "--no-build" "" "BFSNOBUILD" "false" \
  "do not build and install any packages"
script_setopt "--install" "" "BFSINSTALL" "false" "perform install stage only"
script_setopt "--clean" "" "BFSCLEAN" "false" "remove working directories"

script_checkopts $*

BFSROOT="$BFSROOT/$BFSTARGET"
BFSBUILDROOT="$BFSBUILDROOT/$BFSTARGET"
fs_abspath "$BFSXCROOT/$BFSTARGET" BFSXCROOT
BFSMAKEOPTS="$BFSMAKEOPTS -j$BFSCORES"

PATH="$PATH:$BFSXCROOT/bin"

message_boldstart "making boot filesystem in $BFSROOT"

build_checktools $BFSTARGET gcc g++ ar as ranlib ld strip
build_setenv $BFSXCROOT $BFSTARGET $BFSDEBUG

[ -d "$BFSROOT" ] || execute "mkdir -p $BFSROOT"
[ -d "$BFSBUILDROOT" ] || execute "mkdir -p $BFSBUILDROOT"
fs_abspath $BFSROOT BFSROOT
fs_abspath $BFSBUILDROOT BFSBUILDROOT

if false BFSNOBUILD; then
  build_packages "bootfs" $BFSPKGDIR $BFSCONFDIR $BFSPATCHDIR $BFSBUILDROOT \
    $BFSROOT $BFSHOST $BFSTARGET "$BFSMAKEOPTS" $BFSINSTALL $PKGn
fi

if true BFSCLEAN; then
  message_start "cleaning up working directories"
  execute "rm -rf $BFSBUILDROOT"
  message_end
fi

fs_getdirsize $BFSROOT BFSSIZE
message_boldend "success, size of the boot filesystem is ${BFSSIZE}kB"

log_clean
