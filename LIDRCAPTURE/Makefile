debug: 
	@g++ -c main.cpp 
	
release: debug
	@g++ main.o -lpcap
	
clean:
	@rm main.o a.out
	
help:
	@echo "debug: \tCompiles main.cpp to main.o (Default Command)"
	@echo "release: Links main.o to libpcap"
	@echo "clean: \tDeletes the main.o and a.out"
	@echo "	Note: An error may be shown if the last command was not 'make release'"
