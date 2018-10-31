/*	WARNING: COPYRIGHT (C) 2017 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#ifdef _WIN32
#	ifndef WINVER				// Allow use of features specific to Windows XP or later.
#		define WINVER 0x0501		// Change this to the appropriate value to target other versions of Windows.
#	endif

#	ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.
#		define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
#	endif

#	ifndef _WIN32_WINDOWS		// Allow use of features specific to Windows 98 or later.
#		define _WIN32_WINDOWS 0x0501 // Change this to the appropriate value to target Windows Me or later.
#	endif

#	ifndef _WIN32_IE			// Allow use of features specific to IE 6.0 or later.
#		define _WIN32_IE 0x0600	// Change this to the appropriate value to target other versions of IE.
#	endif
#	ifndef WIN32_LEAN_AND_MEAN
#		define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#	endif
#	include <windows.h>
#else
#endif
#include "xstypesdynlib.h"
#include "xstime.h"
#include "xsversion.h"
//#include "buildversion.h"

#ifdef _MANAGED
#pragma managed(push, off)
#endif

// The so_* functions are also used by programs that use xstypes as a static library
// When the function contents change, check if this functionality is also required in these programs
void
#ifdef __GNUC__
__attribute__((constructor(65535)))
#endif
so_init(void)
{
	XsTime_initializeTime();
}

void
#ifdef __GNUC__
__attribute__((destructor))
#endif
so_fini(void)
{
}

#ifdef _WIN32
HMODULE g_hModule = 0;
BOOL APIENTRY DllMain( HMODULE hModule,
					   DWORD  ul_reason_for_call,
					   LPVOID lpvReserved
					 )
{
	g_hModule = hModule;

	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		so_init();
		break;
	case DLL_THREAD_ATTACH:
		break;
	case DLL_THREAD_DETACH:
		break;
	case DLL_PROCESS_DETACH:
		// The lpvReserved parameter to DllMain is NULL if the DLL is being unloaded because of a call to FreeLibrary, it's non NULL if the DLL is being unloaded due to process termination.
		// in case of process termination, all the threads have already been destroyed and we'll just assume that windows will clean us up
		if (lpvReserved == NULL)
			so_fini();		//lint !e522
		break;
	}
	return TRUE;
}
#endif

/*! \brief Return the path that the xsensdeviceapi.dll file is currently running from
	\param path Storage for the path
	\note This is a windows-only function
*/
XSTYPES_DLL_API void xstypesPath(XsString* path)
{
#ifdef XSENS_WINDOWS
	wchar_t filename[1024] = L"";
	if (!path)
		return;

	if (!GetModuleFileNameW(g_hModule, filename, 1024))
		XsString_destruct(path);
	else
		XsString_assignWCharArray(path, filename);
#else
	if (!path)
		return;
	XsString_destruct(path);
#endif
}
#ifdef _MANAGED
#pragma managed(pop)
#endif
