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

# Create a cross compile environment
# See usage for a description of the arguments

. ./functions.sh

BLDPKGS="$BLDPKGS linux-headers binutils gcc-min glibc gcc"
MAKEOPTS="$MAKEOPTS"

init "make a cross compiling environment" "PKGn" "$BLDPKGS" \
  "list of packages to be added to the environment"

set_arg "--root" "DIR" "XCROOT" ".xcomp.root" \
  "root of cross compiling environment"
set_arg "--build-root" "DIR" "BUILDROOT" ".xcomp.build" \
  "temporary build root"
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
set_arg "--no-build" "" "NOBUILD" "false" \
  "do not build and install any packages"
set_arg "--install" "" "INSTALL" "false" \
  "perform install stage only"
set_arg "--clean" "" "CLEAN" "false" \
  "remove working directories"

check_args $*

abs_path $BUILDROOT
BUILDROOT="$ABSPATH/$TARGET"
XCROOT="$XCROOT/$TARGET"
MAKEOPTS="$MAKEOPTS -j$CORES"

message "making cross compile environment in $XCROOT"

execute "mkdir -p $XCROOT"
execute "mkdir -p $BUILDROOT"

if [ "$NOBUILD" != "true" ]; then
  build_packages xcomp $XCROOT "/usr" $BUILDROOT "$MAKEOPTS" $PKGDIR \
    $PATCHDIR $CFGDIR $HOST $TARGET $INSTALL $PKGn
fi

if [ "$CLEAN" == "true" ]; then
  clean $BUILDROOT $ROOT $LOGFILE
else
  clean $LOGFILE
fi

message "success"
