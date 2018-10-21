/*
* Copyright (c) 1999 - 2005 NetGroup, Politecnico di Torino (Italy)
* Copyright (c) 2005 - 2006 CACE Technologies, Davis (California)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the Politecnico di Torino, CACE Technologies
* nor the names of its contributors may be used to endorse or promote
* products derived from this software without specific prior written
* permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <pcap/pcap.h>
//Standard library
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <fstream>
#include <cmath> // unused?
#include <cstring> // unused?
#include <ctime> // unused

//TODO: Move to a header file or (after refactoring out some other functions) move definitions
#pragma region "FUNCTION PROTOTYPES"
/* Takes in the 2 byte values for the azimuth or distance value and returns the calculated value as an integer*/
int TwoByteHexConv (int);
/* Takes in the 4 byte values for the time stamp and returns the calculated value as an integer*/
int FourByteHexConv (int);
#pragma endregion

struct SetupResults {
	bool goodStart = true;
	pcap_t* fp;
};

SetupResults setup (int arg_count, char* args[ ]) {
	SetupResults results;

	char errbuf[PCAP_ERRBUF_SIZE];

	printf ("pktdump_ex: prints the packets of the network using WinPcap.\n");
	printf ("   Usage: pktdump_ex [-s source]\n\n"
			"   Examples:\n"
			"      pktdump_ex -s file://c:/temp/file.acp\n"
			"      pktdump_ex -s rpcap://\\Device\\NPF_{C8736017-F3C3-4373-94AC-9A34B7DAD998}\n\n");

	if (arg_count < 3) {
	
		printf ("\nNo adapter selected: printing the device list:\n");
		/* The user didn't provide a packet source: Retrieve the local device list */
		pcap_if_t *alldevs;
		if (pcap_findalldevs (&alldevs, errbuf) == -1) {
			fprintf (stderr, "Error in pcap_findalldevs_ex: %s\n", errbuf);
			results.goodStart = false;
			return results;
		}

		unsigned int i = 0;
		/* Print the list */
		auto d = alldevs;
		for (; d; d = d->next) {
			printf ("%d. %s\n    ", ++i, d->name);

			if (d->description) {
				printf (" (%s)\n", d->description);
			} else {
				printf (" (No description available)\n");
			}
		}

		if (i == 0) {
			fprintf (stderr, "No interfaces found! Exiting.\n");
			results.goodStart = false;
			return results;
		}

		unsigned int inum = 0;
		printf ("Enter the interface number (1-%d):", i);
		scanf ("%d", &inum);

		if (inum < 1 || inum > i) {
			printf ("\nInterface number out of range.\n");

			/* Free the device list */
			pcap_freealldevs (alldevs);
			results.goodStart = false;
			return results;
		}

		/* Jump to the selected adapter */
		for (d = alldevs; i < inum - 1; d = d->next, i++);
		/* Open the device */
		printf ("%s", d->name);
		if (( results.fp = pcap_open_live (d->name,
										   100 /*snaplen*/,
										   1,
										   20 /*read timeout*/,
										   errbuf)
			 ) == NULL) {
			printf ("%s", errbuf);
			fprintf (stderr, "\nError opening adapter\n");
			results.goodStart = false;
			return results;
		}
	} else {
		// Do not check for the switch type ('-s')
		if (( results.fp = pcap_open_live (args[2],
										   100 /*snaplen*/,
										   1,
										   20 /*read timeout*/,
										   errbuf)
			 ) == NULL) {
			fprintf (stderr, "\nError opening source: %s\n", errbuf);
			results.goodStart = false;
			return results;
		}
	}

	results.goodStart = true;
	return results;
}

int main (int argc, char *argv[ ]) {
	using namespace std;

	auto setupRes = setup (argc, argv);

	//If the setup is bad, kill now
	if (!setupRes.goodStart) {
		return -1;
	}

	/*stores the azimuth value for current block being processed*/
	int azimuth = 0;

	/*Declaration and initialization of the output file that we will be writing to and the input file we will be reading settings from.*/
	ofstream capFile ("LIDAR_data.txt");
	printf ("\n\nvx %i\n\n", azimuth);

	struct pcap_pkthdr *header;
	const u_char *pkt_data;

	int dataBlockStatus = 0;	//used to facilitate
	int blockCounter = 0; //counter for number of data blocks counted in a packet
	/*counter for the number of distance and reflectivity data points processed.*/
	int ctr = 0;
	bool flag = false;
	bool gpsHeader = false;
	int gpsByte = 0;
	int res;
	while (( res = pcap_next_ex (setupRes.fp, &header, &pkt_data) ) >= 0) {
		if (res == 0) //if there is a timeout, continue to the next loop
			continue;

		//TODO: Extract to a function
		for (int i = 1; i < ( header->caplen + 1 ); i++) {
			auto curByte = pkt_data[i - 1];	// The current byte being processed
			auto nextByte = pkt_data[i];	//used in conjunction with curByte

			switch (dataBlockStatus) {
				case 0:	//0xFFEE has not been found, GPS sentence has not been found
					if (curByte == 0xFF && nextByte == 0xEE) {	//detects 0xFFEE
						dataBlockStatus = 1;
						blockCounter++;
					} else if (curByte == '$' && nextByte == 'G') {	//detects start of GPS sentence, "$G"
						dataBlockStatus = 4;
					}
					break;
				case 1: //0xFFEE has been found, begin reading and calculating azimuth value
					if (!flag) {
						/*the purpose of this if statement is to skip one iteration of the for loop. in the previous loop, nextByte
						was used to identify the block flag. In the loop after, that byte became curByte and the azimuth calculation
						begins at the byte AFTER that one. hopefully that made sense.*/
						flag = true;
					} else {
						azimuth = TwoByteHexConv (curByte);

						if (azimuth != -1) {
							capFile << endl << "angle= " << setw (10) << azimuth << " ";
							dataBlockStatus = 2;
						}
					}
					break;
				case 2:	//Azimuth value has been read. Now process the next 32 3-byte data points.
					flag = false;

					ctr++;	//keeps track of how many bytes have been read within this switch case.
							//3 bytes per data point * 32 data points = 96 bytes total. this will be used for the logic.

					if (ctr % 3 != 0) {
						/*temporarily stores the distance value for the current block being processed. Each of the 32 values per block get printed immediately.*/
						auto distance = 2 * TwoByteHexConv (curByte); //multiplied by 2 because the precision is down to 2 millimeters
						if (distance > -1) {
							capFile << " " << setw (10) << distance;
						}
						distance = -1;
					} else {
						capFile << " " << setw (10) << curByte; //reflectivity value
					}

					if (ctr == 96) {
						if (blockCounter == 12) {
							dataBlockStatus = 3;
						} else {
							dataBlockStatus = 0;
						}
						ctr = 0;
					}
					break;
				case 3:	//all 12 blocks in this packet have been read, now process the timestamp and reset dataBlockStatus
				{
					auto timeStamp = FourByteHexConv (curByte);

					if (timeStamp != -1) {
						capFile << endl << "time= " << timeStamp;
						dataBlockStatus = 0;
						blockCounter = 0;
					}
				}
				break;
				case 4:	//Read and immediately print the GPS sentence to the output
					int cB = curByte; //Does curbyte need to be a int?
					cout << endl;

					if (!gpsHeader) {
						capFile << "GPS= $G" << flush;
						gpsHeader = true;
						gpsByte = 84;
					} else if (gpsByte > 0) {
						capFile << static_cast<char>( cB ) << flush;
						gpsByte--;
					} else {
						gpsHeader = false;
						dataBlockStatus = 0;
					}

					break;
			}
		}
	}

	capFile.close ( );
	return 0;
}

#pragma region "globals for hex"
/*
	Used to store hex values in the both hex conversion functions.
	Both functions use them, but set them to 0 before returning
*/
int hex0 = 0;
int hex1 = 0;
int hex2 = 0;
int hex3 = 0;
int hex4 = 0;
int hex5 = 0;
int hex6 = 0;
int hex7 = 0;
//Used to seemingly change modes of the hex conversion funcitions
//Implementation suggests two calls to the conversion functions are needed to return the actual value
int hexMode = 0;
#pragma endregion

int TwoByteHexConv (int hexVal) {
	int val = 0;

	switch (hexMode) {
		case 0:
			//cout << hexVal << " : ";
			hex1 = floor (hexVal / 16);
			//cout << hex1;
			hex0 = hexVal - ( hex1 * 16 );
			//cout << "  " << hex0 << endl;
			hexMode++;
			break;
		case 1:
			//cout << hexVal << " : ";
			hex3 = floor (hexVal / 16);
			//cout << hex3;
			hex2 = hexVal - ( hex3 * 16 );
			//cout << "  " << hex2 << endl;

			val = ( hex3 * pow (16, 3) ) + ( hex2 * pow (16, 2) ) + ( hex1 * 16 ) + hex0;
			hex0 = hex1 = hex2 = hex3 = 0;
			hexMode = 0;
			return val;
	}
	return -1;
}

int FourByteHexConv (int hexVal) {
	int val = 0;

	switch (hexMode) {
		case 0:
			//cout << "case 0; " << hexVal << endl;
			hex1 = floor (hexVal / 16);
			hex0 = hexVal - ( hex1 * 16 );
			// << "   hex1, hex0: " << hex1 << ", " << hex0 << endl;
			hexMode++;
			break;
		case 1:
			//cout << "case 1; " << hexVal << endl;
			hex3 = floor (hexVal / 16);
			hex2 = hexVal - ( hex3 * 16 );
			//cout << "   hex3, hex2: " << hex3 << ", " << hex2 << endl;
			hexMode++;
			break;
		case 2:
			//cout << "case 2; " << hexVal << endl;
			hex5 = floor (hexVal / 16);
			hex4 = hexVal - ( hex5 * 16 );
			//cout << "   hex5, hex4: " << hex5 << ", " << hex4 << endl;
			hexMode++;
			break;
		case 3:
			//cout << "case 3; " << hexVal << endl;
			hex7 = floor (hexVal / 16);
			hex6 = hexVal - ( hex7 * 16 );
			//cout << "   hex7, hex6: " << hex7 << ", " << hex6 << endl;
			hexMode++;
			break;
		case 4:
			//cout << hex7 << ", " << hex6 << ", " << hex5 << ", " << hex4 << ", " << hex3 << ", " << hex2 << ", " << hex1 << ", " << hex0 << endl;
			val = ( hex7 * pow (16, 7) ) + ( hex6 * pow (16, 6) ) + ( hex5 * pow (16, 5) ) + ( hex4 * pow (16, 4) )
				+ ( hex3 * pow (16, 3) ) + ( hex2 * pow (16, 2) ) + ( hex1 * 16 ) + hex0;
			hex7 = hex6 = hex5 = hex4 = hex3 = hex2 = hex1 = hex0 = 0;
			hexMode = 0;
			return val;
	}

	return -1;
}
