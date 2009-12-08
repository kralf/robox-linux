#	Makefile for TDRV001 device driver example
#
CC=gcc

INCLUDEDIR = /usr/include


CFLAGS = -I$(INCLUDEDIR) -Wall -O2

example: example.c ../tdrv001.h
	$(CC) example.c $(CFLAGS) -o example
