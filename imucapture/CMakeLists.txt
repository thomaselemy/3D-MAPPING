cmake_minimum_required(VERSION 3.5.1)
project(imucapture)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(include/xcommunication)
include_directories(include/xsens)
include_directories(src/xstypes)

find_library(LIBDL
        NAMES libdl.so
        PATHS /usr/lib/)

find_library(PTHREAD
        NAMES libpthread.so.0
        PATHS /usr/lib/)

find_library(NCURSES
        NAMES libncurses.so.5
        /usr/lib/)

find_library(TINFO
        NAMES libtinfo.so.5
        /usr/lib/)

set(xcommunication_HEADERS
        include/xcommunication/enumerateusbdevices.h
        include/xcommunication/int_xsdatapacket.h
        include/xcommunication/iointerface.h
        include/xcommunication/iointerfacefile.h
        include/xcommunication/iprotocolhandler.h
        include/xcommunication/legacydatapacket.h
        include/xcommunication/messagelocation.h
        include/xcommunication/mtwsdidata.h
        include/xcommunication/packetfixeddata.h
        include/xcommunication/packetstamper.h
        include/xcommunication/protocolhandler.h
        include/xcommunication/rx_tx_log.h
        include/xcommunication/serialinterface.h
        include/xcommunication/streaminterface.h
        include/xcommunication/synclinegmt.h
        include/xcommunication/synclinemk4.h
        include/xcommunication/usbinterface.h
        include/xcommunication/xcommunicationconfig.h
        include/xcommunication/xslibusb.h
        include/xcommunication/xsudev.h
        include/xcommunication/xswinusb.h)

set(xsens_HEADERS
        include/xsens/pstdint.h
        include/xsens/xsanalogindata.h
        include/xsens/xsarray.h
        include/xsens/xsatomicint.h
        include/xsens/xsbaud.h
        include/xsens/xsbaudcode.h
        include/xsens/xsbaudrate.h
        include/xsens/xsbusid.h
        include/xsens/xsbytearray.h
        include/xsens/xscalibrateddata.h
        include/xsens/xscontrolline.h
        include/xsens/xscopy.h
        include/xsens/xsdataformat.h
        include/xsens/xsdataidentifier.h
        include/xsens/xsdataidentifiervalue.h
        include/xsens/xsdatapacket.h
        include/xsens/xsdebugcounters.h
        include/xsens/xsdeviceid.h
        include/xsens/xsdeviceidarray.h
        include/xsens/xsdeviceidlist.h
        include/xsens/xsdid.h
        include/xsens/xseuler.h
        include/xsens/xsexception.h
        include/xsens/xsfile.h
        include/xsens/xsfilepos.h
        include/xsens/xsgpspvtdata.h
        include/xsens/xsinforequest.h
        include/xsens/xsint64array.h
        include/xsens/xsintarray.h
        include/xsens/xsintlist.h
        include/xsens/xslibraryloader.h
        include/xsens/xsmalloc.h
        include/xsens/xsmath.h
        include/xsens/xsmatrix.h
        include/xsens/xsmatrix3x3.h
        include/xsens/xsmessage.h
        include/xsens/xsmessagearray.h
        include/xsens/xsmessagelist.h
        include/xsens/xsmutex.h
        include/xsens/xsoutputconfiguration.h
        include/xsens/xsoutputconfigurationarray.h
        include/xsens/xsoutputconfigurationlist.h
        include/xsens/xsoutputmode.h
        include/xsens/xsoutputsettings.h
        include/xsens/xsplatform.h
        include/xsens/xsportinfo.h
        include/xsens/xsportinfoarray.h
        include/xsens/xsportinfolist.h
        include/xsens/xspressure.h
        include/xsens/xsquaternion.h
        include/xsens/xsrange.h
        include/xsens/xsrawgnsspvtdata.h
        include/xsens/xsrawgnsssatinfo.h
        include/xsens/xsrawgpsdop.h
        include/xsens/xsrawgpssol.h
        include/xsens/xsrawgpssvinfo.h
        include/xsens/xsrawgpstimeutc.h
        include/xsens/xsresultvalue.h
        include/xsens/xsrssi.h
        include/xsens/xsscrdata.h
        include/xsens/xssdidata.h
        include/xsens/xssimpleversion.h
        include/xsens/xssnapshot.h
        include/xsens/xssocket.h
        include/xsens/xsstring.h
        include/xsens/xsstringarray.h
        include/xsens/xsstringstreaming.h
        include/xsens/xssyncfunction.h
        include/xsens/xssyncline.h
        include/xsens/xssyncpolarity.h
        include/xsens/xssyncrole.h
        include/xsens/xssyncsetting.h
        include/xsens/xssyncsettingarray.h
        include/xsens/xssyncsettinglist.h
        include/xsens/xsthread.h
        include/xsens/xstime.h
        include/xsens/xstimestamp.h
        include/xsens/xstriggerindicationdata.h
        include/xsens/xstypedefs.h
        include/xsens/xstypesconfig.h
        include/xsens/xstypesdynlib.h
        include/xsens/xsushortvector.h
        include/xsens/xsutctime.h
        include/xsens/xsvector.h
        include/xsens/xsvector3.h
        include/xsens/xsversion.h
        include/xsens/xsxbusmessageid.h)

set(xcommunication_SOURCES
        src/xcommunication/enumerateusbdevices.cpp
        src/xcommunication/int_xsdatapacket.cpp
        src/xcommunication/iointerface.cpp
        src/xcommunication/iointerfacefile.cpp
        src/xcommunication/legacydatapacket.cpp
        src/xcommunication/mtwsdidata.cpp
        src/xcommunication/packetfixeddata.cpp
        src/xcommunication/packetstamper.cpp
        src/xcommunication/protocolhandler.cpp
        src/xcommunication/serialinterface.cpp
        src/xcommunication/synclinegmt.c
        src/xcommunication/synclinemk4.c
        src/xcommunication/usbinterface.cpp
        src/xcommunication/xslibusb.cpp
        src/xcommunication/xsudev.cpp
        src/xcommunication/xswinusb.cpp)

set(xstypes_SOURCES
        src/xstypes/datapacket_p.cpp
        src/xstypes/datapacket_p.h
        src/xstypes/resource.h
        src/xstypes/xsarray.c
        src/xstypes/xsbaud.c
        src/xstypes/xsbytearray.c
        src/xstypes/xscalibrateddata.c
        src/xstypes/xsdatapacket.cpp
        src/xstypes/xsdeviceid.c
        src/xstypes/xsdeviceidarray.c
        src/xstypes/xseuler.c
        src/xstypes/xsfile.c
        src/xstypes/xsgpspvtdata.c
        src/xstypes/xsint64array.c
        src/xstypes/xsintarray.c
        src/xstypes/xslibraryloader.c
        src/xstypes/xsmalloc.c
        src/xstypes/xsmath.c
        src/xstypes/xsmatrix.c
        src/xstypes/xsmatrix3x3.c
        src/xstypes/xsmessage.c
        src/xstypes/xsmessagearray.c
        src/xstypes/xsoutputconfiguration.c
        src/xstypes/xsoutputconfigurationarray.c
        src/xstypes/xsportinfo.c
        src/xstypes/xsportinfoarray.c
        src/xstypes/xsquaternion.c
        src/xstypes/xsrange.c
        src/xstypes/xsresultvalue.c
        src/xstypes/xsrssi.c
        src/xstypes/xssdidata.c
        src/xstypes/xssimpleversion.c
        src/xstypes/xssnapshot.c
        src/xstypes/xssocket.c
        src/xstypes/xsstring.c
        src/xstypes/xsstringarray.c
        src/xstypes/xssyncsetting.c
        src/xstypes/xssyncsettingarray.c
        src/xstypes/xsthread.c
        src/xstypes/xstime.c
        src/xstypes/xstimestamp.c
        src/xstypes/xstriggerindicationdata.c
        src/xstypes/xstypedefs.c
        src/xstypes/xstypesdynlib.c
        src/xstypes/xsutctime.c
        src/xstypes/xsvector.c
        src/xstypes/xsvector3.c
        src/xstypes/xsversion.c)

add_executable(imucapture
        ${xcommunication_HEADERS}
        ${xcommunication_SOURCES}
        ${xsens_HEADERS}
        ${xstypes_SOURCES}
        include/deviceclass.h
        include/conio.h
        src/deviceclass.cpp
        src/conio.c
        src/main.cpp)

target_link_libraries(imucapture ${LIBDL} ${PTHREAD} ${NCURSES} ${TINFO})
