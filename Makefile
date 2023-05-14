all:
	colcon build --symlink-install

clean:
	rm -rf build install log

srv:
	@echo ". install/setup.bash"
	ros2 run science_package science_srv

cli:
	@echo ". install/setup.bash"
	ros2 run science_package science_cli $(c) $(t)

pico_pub:
	@echo ". install/setup.bash"
	ros2 run pmt_oscilloscope pico_pub

pico_sub:
	@echo ". install/setup.bash"
	ros2 run pmt_oscilloscope pico_sub

record:
	python3 uv_camera/camera.py