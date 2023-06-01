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


pub:
	ros2 run uv_camera pub

sub:
	ros2 run uv_camera sub

record:
	python3 uv_camera/uv_camera/camera.py


copy:
	scp -r /home/jasper/dev/science_2022_23/ ubuntu@nyx.local:/home/ubuntu/