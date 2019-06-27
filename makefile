

all:
	cd module/hello_world; echo "I'm in module/hello_world"; \
	make
	cd exe/HIL/sdt_sample_code/; echo "I'm in exe/HIL/sdt_sample_code/"; \
	trick-CP
run:
	cd exe/HIL/sdt_sample_code/; echo "I'm in exe/HIL/sdt_sample_code/"; \
	./S_main_Linux_7_x86_64.exe RUN_test/input.cpp

clean:
	cd module/hello_world; echo "I'm in module/hello_world"; \
	make clean
	cd exe/HIL/sdt_sample_code/; echo "I'm in exe/HIL/sdt_sample_code/"; \
	make clean