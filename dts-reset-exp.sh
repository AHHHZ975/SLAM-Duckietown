dts duckiebot virtual stop vquackgpt
dts duckiebot virtual start vquackgpt
sleep 20
dts code build -R vquackgpt
docker rm dts-start-matrix
dts code start_matrix -R vquackgpt
