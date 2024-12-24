#!/bin/sh

dts duckiebot virtual stop vquackgpt
dts duckiebot virtual start vquackgpt
sleep 20
dts code build -R vquackgpt
docker rm dts-start-matrix

echo "Virtual Duckiebot startedm, code built and dts-start-matrix docker removed."
