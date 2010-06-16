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

# Create a Debian root filesystem from scratch
# See usage for a description of the arguments

. ubash

script_init_array "Create a Debian root filesystem from scratch" \
  "PKG" DEBPKGS "$DEBPKGS" "list of packages to be added to the Debian image"

script_setopt "--image|-i" "FILE" "DEBIMAGE" "rootfs.debian.img" \
  "root filesystem image to be created"
script_setopt "--size" "SIZE" "DEBSIZE" "128" \
  "size of the root filesystem in MB"
script_setopt "--target" "i686|powerpc|..." "DEBTARGET" "powerpc" \
  "target architecture"
script_setopt "--dist" "woody|sarge|..." "DEBDIST" "sarge" \
  "Debian distribution"
script_setopt "--mirror" "us|de|..." "DEBMIRROR" "de" \
  "package download mirror"
script_setopt "--mount-point" "DIR" "DEBMNT" ".rootfs.debian.mount" \
  "temporary mount point of root filesystem"

script_checkopts $*
script_checkroot

SERVER="http://ftp.$DEBMIRROR.debian.org/debian"
if [ ${#DEBPKGS[*]} != 0 ]; then
  DEBPKGS=${DEBPKGS[*]}
  DEBPKGS=${DEBPKGS//" "/","}
  DEBINCLUDES="--include=$DEBPKGS"
fi

message_boldstart "making Debian root filesystem image $DEBIMAGE"

message_start "creating zeroed root filesystem image $DEBIMAGE"
execute "dd if=/dev/zero of=$DEBIMAGE bs=1M count=$DEBSIZE"
execute "/sbin/mkfs -F $DEBIMAGE"
message_end

message_start "mounting root filesystem image to $DEBMNT"
execute "mkdir $DEBMNT"
execute "mount -o loop $DEBIMAGE $DEBMNT"
message_end

message_start "processing package list"
DEBOPTS="$DEBTARGET $DEBINCLUDES $DEBDIST $DEBMNT $DEBSERVER"
execute "debootstrap --verbose --arch $DEBOPTS"
message_end

message_start "unmounting root filesystem image"
execute "umount $DEBMNT"
execute "rm -r $DEBMNT"
message_end

message_boldend "success"
