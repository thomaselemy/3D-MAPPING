debug: 
	@g++ -c -o lidar.o main_lidar.cpp 
	
release: debug
	@g++ lidar.o -lpcap
	
clean:
	@rm *.o a.out
	@clear
	
help:
	@echo "debug: \tCompiles main.cpp to main.o (Default Command)"
	@echo "release: Links main.o to libpcap"
	@echo "clean: \tDeletes the main.o and a.out"
	@echo "	Note: An error may be shown if the last command was not 'make release'"
