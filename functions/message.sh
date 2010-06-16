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

# Message functions

if ! defined STAGE; then
  STAGE=0
  export STAGE
fi

function message
{
  for (( S=0; S < $STAGE; S++ )); do
    echo -n "    "
  done

  echo -n "|-> "
  echo -e $1
}

function message_start
{
  message "$1"
  message_stageup
}

function message_end
{
  message_stagedown
  [ -n "$1" ] && message "$1"
}

function message_bold
{
  message "\033[1m$1\033[0m"
}

function message_boldstart
{
  message_bold "$1"
  message_stageup
}

function message_boldend
{
  message_stagedown
  [ -n "$1" ] && message_bold "$1"
}

function message_warn
{
  message "\033[33mwarning:\033[0m $1"
}

function message_exit
{
  message "\033[31merror:\033[0m $1"

  STAGE=0
  message "bailing out, see $LOGFILE for details"

  [ -n "$SCRIPT" ] &&  exit 1
}

function message_confirm
{
  CONFIRM=""

  message "\033[1mconfirm:\033[0m $1? [Y/n]"

  while ! [[ "$CONFIRM" =~ [Yn] ]]; do
    read -s -n 1 CONFIRM
  done

  [ "$CONFIRM" == "Y" ] && define $2 "true" || define $2 "false"
}

function message_abort
{
  message_confirm "$1, continue" RETVAL

  if ! true RETVAL; then
    STAGE=0
    message "user abort"
  
    exit 2
  fi
}

function message_stageup
{
  math_inc STAGE
}

function message_stagedown
{
  [[ $STAGE > 0 ]] && math_dec STAGE
}
