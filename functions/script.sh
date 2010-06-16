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

CHARWIDTH=30

SCRIPTMAINTAINER="ralf.kaestner@gmail.com"
[ "$0" != "$SHELL" ] && SCRIPT=`basename $0` || unset SCRIPT
unset SCRIPTDOC
unset SCRIPTARGVAR
unset SCRIPTARGDEF
unset SCRIPTARGDOC

SCRIPTOPTTAGS=("--help"
               "--verbose|-v"
               "--logfile")

SCRIPTOPTVALS=(""
               ""
               "FILE")

SCRIPTOPTVARS=("HELP"
               "VERBOSE"
               "LOGFILE")

SCRIPTOPTDEFS=("false"
               "false"
               "`basename $0 .sh`.log")

SCRIPTOPTDOCS=("display usage and exit"
               "generate verbose command output"
               "temporary log file")

function script_init
{
  SCRIPTDOC=$1
  SCRIPTARGVAR=$2
  SCRIPTARGDEF=$3
  SCRIPTARGDOC=$4
}

function script_print
{
  NUMCHARS=`echo -n "$1" | wc -m`
  math_calc "$CHARWIDTH-$NUMCHARS" NUMBLANKS

  echo -n "  $1"

  for (( B=0; B < $NUMBLANKS; B++ )); do
    echo -n " "
  done

  echo "$2"
}

function script_usage
{
  echo -n "usage: $SCRIPT [OPT1 OPT2 ...]"

  if [ -n "$SCRIPTARGVAR" ]; then
    if [[ "$SCRIPTARGVAR" =~ n$ ]]; then
      SCRIPTDSPVAR=`echo $SCRIPTARGVAR | sed s/n$//`
      echo " [${SCRIPTDSPVAR}1 ${SCRIPTDSPVAR}2 ...]"
    else
      echo " [${SCRIPTARGVAR}]"
    fi
  else
    echo ""
  fi

  [ -n "$SCRIPTDOC" ] && echo -e "\033[1m$SCRIPTDOC\033[0m"

  if [ -n "$SCRIPTARGVAR" ]; then
    if [[ "$SCRIPTARGVAR" =~ n$ ]]; then
      SCRIPTDSPVAR=`echo $SCRIPTARGVAR | sed s/n$//`
      script_print "${SCRIPTDSPVAR}1 ${SCRIPTDSPVAR}2 ..." "$SCRIPTARGDOC"
    else
      script_print "${SCRIPTARGVAR}" "$SCRIPTARGDOC"
    fi
  fi

  script_print "OPT1 OPT2 ..." "list of options as given below [default]"

  for (( A=0; A < ${#SCRIPTOPTTAGS[@]}; A++ )); do
    OPTTAG=${SCRIPTOPTTAGS[A]}
    OPTTAG=${OPTTAG//"|"/", "}
    OPTVAL=${SCRIPTOPTVALS[A]}
    OPTVAR=${SCRIPTOPTVARS[A]}
    OPTDEF=${SCRIPTOPTDEFS[A]}
    OPTDOC=${SCRIPTOPTDOCS[A]}

    if [ -n "${SCRIPTOPTDEFS[A]}" ]; then
      script_print "$OPTTAG $OPTVAL" "$OPTDOC [$OPTDEF]"
    else
      script_print "$OPTTAG $OPTVAL" "$OPTDOC"
    fi
  done

  echo "Report bugs to <$SCRIPTMAINTAINER>, attach error logs"
}

function script_setopt
{
  NUMOPTS=${#SCRIPTOPTTAGS[*]}

  SCRIPTOPTTAGS[$NUMOPTS]=$1
  SCRIPTOPTVALS[$NUMOPTS]=$2
  SCRIPTOPTVARS[$NUMOPTS]=$3
  SCRIPTOPTDEFS[$NUMOPTS]=$4
  SCRIPTOPTDOCS[$NUMOPTS]=$5
}

function script_checkopts
{
  unset SCRIPTARGS

  NUMOPTS=${#SCRIPTOPTTAGS[*]}
  for (( A=0; A < $NUMOPTS; A++ )); do
    define ${SCRIPTOPTVARS[A]} ${SCRIPTOPTDEFS[A]}
  done

  RETVAL=0

  while [ -n "$1" ]; do
    if [[ "$1" =~ ^- ]]; then
      MATCH="false"

      for (( A=0; A < $NUMOPTS; A++ )); do
        OPTTAG=${SCRIPTOPTTAGS[A]}
        OPTVAL=${SCRIPTOPTVALS[A]}
        OPTVAR=${SCRIPTOPTVARS[A]}
        OPTDEF=${SCRIPTOPTDEFS[A]}
        OPTDOC=${SCRIPTOPTDOCS[A]}

        if [[ "$1" =~ $OPTTAG ]]; then
          if [[ "$OPTDEF" =~ ^true$|^false$|^yes$|^no$ ]]; then
            define $OPTVAR "true"
            MATCH="true"
          else
            define $OPTVAR $2
            MATCH="true"
            shift
          fi
        fi
      done

      if ! true MATCH; then
        echo "unknown argument: $1"
        RETVAL=1
        HELP="true"
      fi
    else
      SCRIPTARGS[${#SCRIPTARGS[*]}]=$1
    fi

    shift
  done

  fs_abspath $LOGFILE LOGFILE
  log_clean

  if [ -n "$SCRIPTARGVAR" ]; then
    if [ ${#SCRIPTARGS[@]} != 0 ]; then
      define $SCRIPTARGVAR "${SCRIPTARGS[@]}"
    else
      if [ -n "$SCRIPTARGDEF" ]; then
        define $SCRIPTARGVAR "${SCRIPTARGDEF[@]}"
      else
        echo "missing argument(s): $SCRIPTARGVAR"
        RETVAL=1
        HELP="true"
      fi
    fi
  fi

  if true HELP; then
    script_usage
    exit $RETVAL
  fi
}

function script_checkroot
{
  [ "$UID" != "0" ] && message_exit "$SCRIPT must be run as root"
}

function script_run
{
  SCRIPTDIR=$1
  shift
  message_start "running scripts in $SCRIPTDIR"

  SCRIPTPWD=`pwd`
  execute "cd $SCRIPTDIR"

  while [ -n "$1" ]; do
    message_start "executing script $1"

    "./$1"
    [ $? != 0 ] && exit $?

    message_end "success, script returned zero"
    shift
  done

  execute "cd $SCRIPTPWD"

  message_end "success, all scripts executed"
}
