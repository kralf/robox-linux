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

# Functions

function check_xcomp
{
  TARGET=$1
  shift

  stage_up
  message "checking the cross compiler"

  while [ "$1" != "" ]; do
    execute "$TARGET-linux-$1 --version"
    shift
  done

  stage_down
}

function set_xcomp
{
  stage_up
  message "setting up environment"

  CPP="$1-linux-cpp"
  CC="$1-linux-gcc"
  CXX="$1-linux-g++"
  AR="$1-linux-ar"
  AS="$1-linux-as"
  RANLIB="$1-linux-ranlib"
  LD="$1-linux-ld"
  STRIP="$1-linux-strip"
  export CC CXX AR AS RANLIB LD STRIP

  CFLAGS="-I$2/usr/include -I$3/include -I$3/usr/include"
  LDFLAGS="-L$2/lib -L$2/usr/lib -L$3/lib -L$3/usr/lib"
  if [ "$4" != "true" ]; then
    LDFLAGS="$LDFLAGS -s"
  fi
  export CFLAGS LDFLAGS

  stage_down
}

function rm_symbols
{
  stage_up
  message "removing symbols links in $1"

  RETDIR=`pwd`
  execute "cd $1"

  execute "find -wholename ./*.so -exec $STRIP -s {} ;"
  execute "find -perm /111 -exec $STRIP -s {} ;"

  execute "cd $RETDIR"

  stage_down
}

function extract_package
{
  EXTRACTPKG=$2

  case ${EXTRACTPKG##*.} in
    tar) execute "tar -xf $EXTRACTPKG -C $1"
         EXTRACTPKG=`basename $EXTRACTPKG .tar`
         ;;
    bz2) execute "tar -xjf $EXTRACTPKG -C $1"
         EXTRACTPKG=`basename $EXTRACTPKG .tar.bz2`
         ;;
     gz) execute "tar -xzf $EXTRACTPKG -C $1"
         EXTRACTPKG=`basename $EXTRACTPKG .tar.gz`
         ;;
    tgz) execute "tar -xzf $EXTRACTPKG -C $1"
         EXTRACTPKG=`basename $EXTRACTPKG .tgz`
         ;;
      *) exit_message "unsupported format of package $EXTRACTPKG"
         ;;
  esac
}

function build_packages
{
  SUFFIX=$1
  shift
  abs_path $1
  ROOT=$ABSPATH
  shift
  USRDIR=$1
  USRROOT=$ROOT$USRDIR
  shift
  BUILDROOT=$1
  shift
  MAKEOPTS=$1
  shift
  abs_path $1
  PGKDIR=$ABSPATH
  shift
  abs_path $1
  PATCHDIR=$ABSPATH
  shift
  abs_path $1
  CFGDIR=$ABSPATH
  shift
  HOST=$1
  shift
  TARGET=$1
  shift
  INSTALL=$1
  shift

  stage_up
  message "processing package list"
  stage_up

  while [ "$1" != "" ]; do
    ALIAS="$1"
    ADDONS=""
    BUILDDIR="."
    DEFCONFIGURE="./configure --prefix=$USRROOT --exec-prefix=$ROOT"
    CONFIGURE=("$DEFCONFIGURE")
    MAKEBUILD=("make $MAKEOPTS all")
    MAKEINSTALL=("make $MAKEOPTS install")
    COMMENT="this may take a while"
    PKG=""
    PKGBUILDROOT=$BUILDROOT/$1

    message "processing package $1"
    stage_up

    if [ -r $CFGDIR/$1.$SUFFIX ]; then
      message "sourcing package configuration"
      . $CFGDIR/$1.$SUFFIX
    fi

    if [ "$PKG" == "" ] || [ ! -r $PKG ]; then
      PKG=`ls $PKGDIR/$ALIAS-[0-9]*.{gz,tgz,bz2} 2> /dev/null`
    fi
    if [ "$PKG" != "" ]; then
      PKG=`readlink -m $PKG`
    fi

    if [ "$PKG" != "" ] && [ -r $PKG ]; then
      PKGBASENAME=`basename $PKG`
      FULLPKGNAME=${PKGBASENAME%.tar*}
      PKGNAME=${FULLPKGNAME%%-[0-9]*}
      PKGVERSION=${FULLPKGNAME##*-}

      if [ -x $PKGBUILDROOT ]; then
        message "contents of $PKGBASENAME found in $PKGBUILDROOT"
      else
        if [ -d $PKG ]; then
          message "linking $PKG to $PKGBUILDROOT"
          execute "ln -sf $PKG $PKGBUILDROOT"
        else
          message "extracting contents of $PKGBASENAME to $PKGBUILDROOT"
          extract_package $BUILDROOT $PKG

          PATCHES=`ls $PATCHDIR/$ALIAS-$PKGVERSION*.patch 2> /dev/null`
          if [ "$PATCHES" != "" ]; then
            message "patching package sources"
            stage_up

            for PATCH in $PATCHES; do
              message "applying $PATCH"
              patch -d $BUILDROOT -p0 < $PATCH >& /dev/null
            done

            stage_down
          fi

          if [ ! -x $PKGBUILDROOT ]; then
            execute "mv $BUILDROOT/$EXTRACTPKG $PKGBUILDROOT"
          fi

          if [ "$ADDONS" != "" ]; then
            message "extracting addons to $PKGBUILDROOT"
            stage_up

            for ADDON in $ADDONS; do
              ADDONPKGS=`ls $PKGDIR/$ADDON-[0-9]*.{gz,tgz,bz2} 2> /dev/null`
              for ADDONPKG in $ADDONPKGS; do
                ADDONPKG=`readlink -m $ADDONPKG`
                ADDONBASENAME=`basename $ADDONPKG`

                message "extracting contents of $ADDONBASENAME to $PKGBUILDROOT"
                extract_package $PKGBUILDROOT $ADDONPKG
              done
            done

            stage_down
          fi
        fi
      fi

      message "descending into build directory"
      RETDIR=`pwd`
      execute "cd $PKGBUILDROOT"
      if [ ! -x $BUILDDIR ]; then
        execute "mkdir -p $BUILDDIR"
      fi
      execute "cd $BUILDDIR"

      if [ "$INSTALL" != "true" ] && [ "$CONFIGURE" != "" ]; then
        message "configuring package sources"
        execute "${CONFIGURE[@]}"
      fi
      if [ "$INSTALL" != "true" ] && [ "$MAKEBUILD" != "" ]; then
        message "compiling package sources ($COMMENT)"
        execute "${MAKEBUILD[@]}"
      fi

      if [ "$MAKEINSTALL" != "" ]; then
        message "installing built package content"
        execute "${MAKEINSTALL[@]}"
      fi

      message "ascending from build directory"
      execute "cd $RETDIR"
    else
      warn_message "package $1 not found"
    fi

    stage_down
    message "package processed"
    shift
  done

  stage_down
  stage_down
}

function mk_image
{
  stage_up
  message "making filesystem image $1"
  stage_up

  message "evaluating image size"
  get_dirsize $2
  calc "$SIZE*1.05+$6"
  SIZE=$RETVAL

  message "writing zeroed image"
  execute "dd if=/dev/zero of=$1 bs=1k count=$SIZE"

  message "building filesystem on image device"
  execute "/sbin/mkfs -t $4 -F $1 -b $5"

  message "mounting filesystem image to $3"
  execute "mkdir -p $3"
  execute "mount -o loop $1 $3"

  message "copying filesystem content"
  execute "cp -a $2/* $3"

  message "unmounting filesystem image"
  execute "umount $3"
  execute "rm -rf $3"

  stage_down
  stage_down
}

function upload_image
{
  message "uploading image $1 to $2"

  execute "scp $1 $3@$2:$4"
}

function boot_image
{
  KERNEL=`ls $3/vmlinux-* 2> /dev/null`
  if [ "$KERNEL" = "" ]; then
    exit_message "missing kernel image in $3"
  else
    message "booting kernel $KERNEL"
    stage_up

    message "creating domain $1"
    abs_path $4
    xm create -c $2 name=$1 kernel=$KERNEL disk=file:$ABSPATH,hda1,w

    if [ "$EXITVAL" != 0 ]; then
      message "failed to create domain $1"
    fi
    stage_down
  fi
}
