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

# Create robox-linux cross compiling environment and boot image
# See usage for a description of the arguments

. ubash

script_init "Create robox-linux cross compiling environment and boot ramdisk"

script_setopt "--host" "i686|powerpc|..." "RLHOST" "`uname -m`" \
  "override host architecture"
script_setopt "--target" "i686|powerpc|..." "RLTARGET" "`uname -m`" \
  "target architecture"
script_setopt "--cores" "NUM" "RLCORES" "1" "number of cores to compile on"
script_setopt "--make-args|-m" "ARGS" "RLMAKEARGS" "" \
  "additional arguments to be passed to make"

script_setopt "--package-dir" "DIR" "RLPKGDIR" "pkg" \
  "directory containing packages"
script_setopt "--config-dir" "DIR" "RLCONFDIR" "conf" \
  "directory containing build configurations"
script_setopt "--patch-dir" "DIR" "RLPATCHDIR" "patch" \
  "directory containing patches"

script_setopt "--tftp-host" "ADDRESS" "RLTFTPHOST" "shantipc0" \
  "ip or dns name of the tftp server"
script_setopt "--tftp-user" "LOGIN" "RLTFTPUSER" "tftp" \
  "login name of the tftp user"
script_setopt "--tftp-root" "DIR" "RLTFTPROOT" "~" \
  "tftp root directory"

script_setopt "--no-build" "" "RLNOBUILD" "false" \
  "do not build and install any packages"
script_setopt "--install" "" "RLINSTALL" "false" "perform install stage only"
script_setopt "--clean" "" "RLCLEAN" "false" "remove working directories"

script_checkopts $*
script_checkroot

RLBRDIMAGE=".bootrd.images/$RLTARGET/bootrd.img"

RLARGS="--host $RLHOST --target $RLTARGET"
RLARGS="$RLARGS --package-dir $RLPKGDIR --config-dir $RLCONFDIR"
true RLNOBUILD && RLARGS="$RLARGS --no-build"
true RLINSTALL && RLARGS="$RLARGS --install"
true RLCLEAN && RLARGS="$RLARGS --clean"

message_boldstart "making robox-linux"

STAGE=$STAGE ./mkxcomp.sh $RLARGS && \
STAGE=$STAGE ./mkrootfs.sh $RLARGS && \
STAGE=$STAGE ./mkbootrd.sh $RLARGS && 
network_upfiles $RLTFTPUSER $RLTFTPHOST $RLTFTPROOT $RLBRDIMAGE 

message_boldend "success"

log_clean
