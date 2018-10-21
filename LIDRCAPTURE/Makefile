debug: 
	g++ -c main.cpp 
	
release: debug
	g++ main.o -lpcap
	
help:
	@echo "Debug (Default): Compiles main.cpp"
	@echo "Release: Links main.o to libpcap"
