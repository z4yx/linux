#!/bin/sh

mount -a
#here mounted /dev

mkdir /dev/pts
mount devpts -t devpts /dev/pts
