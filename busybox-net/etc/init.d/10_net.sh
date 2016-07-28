#!/bin/sh

ip a add 172.16.0.20/24 dev eth0
ip l set eth0 up
