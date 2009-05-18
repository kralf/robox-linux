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

# Create a cross compile environment
# See usage for a description of the arguments

. functions/global.sh

XCPKGS="linux-headers binutils glibc-headers gcc-min glibc gcc"

script_init "make a cross compiling environment" "PKGn" "$XCPKGS" \
  "list of packages to be added to the environment"

script_setopt "--root" "DIR" "XCROOT" ".xcomp.root" \
  "root of cross compiling environment"
script_setopt "--build-root" "DIR" "XCBUILDROOT" ".xcomp.build" \
  "temporary build root"
script_setopt "--host" "i686|powerpc|..." "XCHOST" "`uname -m`" \
  "override host architecture"
script_setopt "--target" "i686|powerpc|..." "XCTARGET" "`uname -m`" \
  "target architecture"
script_setopt "--cores" "NUM" "XCCORES" "1" "number of cores to compile on"
script_setopt "--make-args|-m" "ARGS" "XCMAKEARGS" "" \
  "additional arguments to be passed to make"
script_setopt "--package-dir" "DIR" "XCPKGDIR" "packages" \
  "directory containing packages"
script_setopt "--config-dir" "DIR" "XCCONFDIR" "configurations" \
  "directory containing build configurations"
script_setopt "--patch-dir" "DIR" "XCPATCHDIR" "patches" \
  "directory containing patches"
script_setopt "--no-build" "" "XCNOBUILD" "false" \
  "do not build and install any packages"
script_setopt "--install" "" "XCINSTALL" "false" "perform install stage only"
script_setopt "--clean" "" "XCCLEAN" "false" "remove working directories"

script_checkopts $*

fs_abspath "$XCBUILDROOT/$XCTARGET" XCBUILDROOT
fs_abspath "$XCROOT/$XCTARGET" XCROOT
XCMAKEOPTS="$XCMAKEOPTS -j$XCCORES"

message_boldstart "making cross compile environment in $XCROOT"

[ -d "$XCROOT" ] || execute "mkdir -p $XCROOT"
[ -d "$XCBUILDROOT" ] || execute "mkdir -p $XCBUILDROOT"

if false XCNOBUILD; then
  build_packages "xcomp" $XCPKGDIR $XCCONFDIR $XCPATCHDIR $XCBUILDROOT $XCROOT \
    $XCHOST $XCTARGET "$XCMAKEOPTS" $XCINSTALL $PKGn
fi

if true XCCLEAN; then
  message_start "cleaning up working directories"
  execute "rm -rf $XCBUILDROOT"
  message_end
fi

fs_getdirsize $XCROOT XCSIZE
message_boldend "success, size of the cross compile environment is ${XCSIZE}kB"

log_clean
