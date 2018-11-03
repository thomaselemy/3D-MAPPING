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
#include <fstream>
#include <cmath> // unused?
#include <cstring> // unused?
#include <ctime> // unused

//TODO: Move to a header file or (after refactoring out some other functions) move definitions
#pragma region "FUNCTION PROTOTYPES"
/* Takes in the 2 byte values for the azimuth or distance value and returns the calculated value as an integer*/
int HexConv (int);
int TwoByteHexConv (int, int);
/* Takes in the 4 byte values for the time stamp and returns the calculated value as an integer*/
int FourByteHexConv (int, int, int, int);
#pragma endregion

struct SetupResults {
	bool goodStart = true;
	pcap_t* fp;
};

SetupResults setup (int arg_count, char* args[ ]) {
	SetupResults results;

	pcap_if_t *alldevs;
	char errbuf[PCAP_ERRBUF_SIZE];

	printf("Hi.\n");

	printf ("pktdump_ex: prints the packets of the network using WinPcap.\n");
	printf ("   Usage: pktdump_ex [-s source]\n\n"
			"   Examples:\n"
			"      pktdump_ex -s file://c:/temp/file.acp\n"
			"      pktdump_ex -s rpcap://\\Device\\NPF_{C8736017-F3C3-4373-94AC-9A34B7DAD998}\n\n");

	if (arg_count < 3) {
		printf ("\nNo adapter selected: printing the device list:\n");
		/* The user didn't provide a packet source: Retrieve the local device list */
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

using namespace std;
//Opens the file stream to LIDAR_data.txt
ofstream capFile ("LIDAR_data.txt");

//UDP header Offset
const int offset = 42;

//Processes our lidar Data
static void process(u_char *args, const struct pcap_pkthdr *header, const u_char *pkt_data) {

	if(header->len == 1248) {
		//values we need
		int azimuth = 0;
		int distance = 0;
		int timeStamp = 0;

		//Loop through the 12 Data Blocks
		for(int i = 0; i < 12; i++) {
			azimuth = TwoByteHexConv(pkt_data[offset + 2], pkt_data[offset + 2 + 1]);

			capFile << "angle=" << setw(10) << azimuth << " ";

			//Loop through the channels;
			for (int j = 0; j < 32; j++) {

				//42   +      (i * 100)  + 4     +       j    +    0/1/2
				//UDP header  data block   flag&azimuth  channel   distance then reflectivity

				distance = TwoByteHexConv(pkt_data[offset + (i * 100) + 4 + j], pkt_data[offset + (i * 100) + 4 + j + 1]);
				capFile << setw(10) << distance << " ";

				capFile << setw(10) << pkt_data[offset + (i * 100) + 4 + j + 2] << " ";
			}
		}

		//time stamp is 4 bytes
		timeStamp = FourByteHexConv(pkt_data[1242], pkt_data[1243], pkt_data[1244], pkt_data[1245]);

		//output the timestamp and endl
		capFile << "time = " << timeStamp << endl;
	} else {

		//GPSRMC sentence is variable length so we need to loop until we hit the CR/LF bytes.
		capFile << "GPS= $G";
		int i = 0;
		while(1) {
			//check for end of GPS sentence
			if(((int)pkt_data[249 + i]) == 0x0d) {
				if(((int)pkt_data[249 + i + 1]) == 0x0a) {
					break;
				}
			}

			//print GPS sentence
			capFile << static_cast<char>(pkt_data[248 + i]);

			i++;
		}

		//end the line
		cout << endl;
	}
}

int main (int argc, char *argv[ ]) {

	auto setupRes = setup (argc, argv);

	//call the setup function, if it we get a proper connection it will set good start to true
	if(setupRes.goodStart) {
		//This is a promise call to pcap. process is our callback. Pcap will call process when it has some data for us.
		pcap_loop(setupRes.fp, 0, process, NULL);
	}

	return 0;
}

#pragma region "globals for hex"
//Used to store hex values in the hex conversion functions. All hex conversions use them

int hex0 = 0;
int hex1 = 0;
int hex2 = 0;
int hex3 = 0;
int hex4 = 0;
int hex5 = 0;
int hex6 = 0;
int hex7 = 0;

#pragma endregion

//Converter functions for converting an integer hex value to decimal
int HexConv(int hexVal) {

	hex1 = floor (hexVal / 16) * 16;
	hex0 = hexVal - ( hex1 * 16 );

	return ( hex1 * 16 ) + hex0;
}

int TwoByteHexConv (int hexVal, int hexVal2) {

	//hexVal
	hex1 = floor (hexVal / 16);
	hex0 = hexVal - ( hex1 * 16 );

	//hexVal2
	hex3 = floor (hexVal2 / 16);
	hex2 = hexVal2 - ( hex3 * 16 );

	return ( hex3 * pow (16, 3) ) + ( hex2 * pow (16, 2) ) + ( hex1 * 16 ) + hex0;
}

int FourByteHexConv (int hexVal, int hexVal2, int hexVal3, int hexVal4) {

	hex1 = floor (hexVal / 16);
	hex0 = hexVal - ( hex1 * 16 );

	//val2
	hex3 = floor (hexVal2 / 16);
	hex2 = hexVal2 - ( hex3 * 16 );

	//val3
	hex5 = floor (hexVal3 / 16);
	hex4 = hexVal3 - ( hex5 * 16 );

	//val4
	hex7 = floor (hexVal4 / 16);
	hex6 = hexVal4 - ( hex7 * 16 );

	return ( hex7 * pow (16, 7) ) + ( hex6 * pow (16, 6) ) + ( hex5 * pow (16, 5) ) + ( hex4 * pow (16, 4) ) + ( hex3 * pow (16, 3) ) + ( hex2 * pow (16, 2) ) + ( hex1 * 16 ) + hex0;
}
